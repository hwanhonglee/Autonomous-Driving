"""HH_260929 - Check local-only depth decoding and explicit semantic provenance."""

import numpy as np
import pytest

from scripts.e2e import audit_learned_mapping_bridge as bridge
from portable_e2e.mapping import Observation, SemanticEvidenceMap


def test_depth_decoding_uses_optical_axis_meters():
    raw = np.zeros((360, 640, 4), dtype=np.uint8)
    raw[0, 0, 2] = 255
    raw[0, 0, 1] = 10
    depth = bridge.decode_depth_m(raw)
    assert depth[0, 0] == pytest.approx(2815 * 1000 / 16777215, rel=1e-6)


def test_oracle_tags_are_only_a_diagnostic_target():
    raw = np.zeros((360, 640, 4), dtype=np.uint8)
    raw[10, 10, 2] = 24
    raw[11, 11, 2] = 1
    target = bridge.semantic_target(raw)
    assert target[10, 10] == 3 and target[11, 11] == 1
    assert target[0, 0] == 0


def test_pixel_metrics_keep_absent_classes_undefined():
    target = np.zeros((360, 640), dtype=np.uint8)
    prediction = np.zeros_like(target)
    target[0, 0] = 3
    prediction[0, 0] = 1
    result = bridge.pixel_metrics(target, prediction)
    assert result["confusion"][3][1] == 1
    assert result["iou"]["lane_marking"] == 0
    assert result["iou"]["pole"] is None


def test_cell_overlap_uses_common_observed_cells_not_unknown_space():
    oracle = SemanticEvidenceMap("carla_ue:Town01", "oracle_sensor", resolution_m=1)
    learned = SemanticEvidenceMap("carla_ue:Town01", "learned_prediction", resolution_m=1)
    oracle.update(Observation(1, oracle.frame_id, oracle.source,
                              ((0.1, 0.1, "road", 1), (1.1, 0.1, "sidewalk", 1))))
    learned.update(Observation(1, learned.frame_id, learned.source,
                               ((0.1, 0.1, "lane_marking", 1),
                                (2.1, 0.1, "road", 1))))
    result = bridge.cell_overlap(oracle, learned)
    assert result["common_cells"] == 1
    assert result["union_cells"] == 3
    assert result["occupancy_jaccard"] == pytest.approx(1 / 3)
    assert result["dominant_label_agreement_in_common_cells"] == 0
