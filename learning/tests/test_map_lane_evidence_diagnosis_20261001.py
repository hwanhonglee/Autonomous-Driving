"""HH_261001 - Independently test accepted-evidence partition, insertion ties and promotion FP accounting."""

import inspect
import numpy as np
import pytest
from scripts.e2e import diagnose_map_lane_evidence_20261001 as study


def cell(order, frames=None, weights=None):
    frames = frames or {i: 1 for i in order}
    weights = weights or {i: float(frames[i]) for i in order}
    return study.Cell(tuple(order), tuple(frames.get(i, 0) for i in range(6)), tuple(weights.get(i, 0.) for i in range(6)))


def test_exact_confidence_ties_follow_insertion_not_label_id_or_frames():
    assert cell([2, 0]).dominant == 2
    assert cell([0, 2]).dominant == 0
    assert cell([2, 0], {2: 2, 0: 3}, {2: 2., 0: 1.5}).dominant == 2


def test_all_four_categories_and_exact_sum():
    oracle = {(i, 0): cell([2]) for i in range(4)}
    learned = {(1, 0): cell([0]), (2, 0): cell([0, 2], {0: 2, 2: 1}), (3, 0): cell([2])}
    result = study.partition(oracle, learned)
    assert [result['four_way'][name]['count'] for name in study.GROUPS] == [1, 1, 1, 1]
    assert result['counts_sum'] == result['oracle_dominant_lane_cells'] == 4
    assert result['accepted_lane_evidence_missing']['among_current_lane_misses']['fraction'] == 2 / 3


def test_false_promotion_risk_and_secondary_oracle_lane_are_distinct():
    oracle = {(0, 0): cell([2]), (1, 0): cell([0, 2], {0: 2, 2: 1}), (2, 0): cell([0])}
    learned = {(0, 0): cell([0, 2], {0: 2, 2: 1}),
        (1, 0): cell([0, 2], {0: 2, 2: 1}), (2, 0): cell([2]), (3, 0): cell([2])}
    risk = study.partition(oracle, learned)['any_accepted_lane_presence_set_diagnostic']
    assert risk['current_dominant']['true_positive_cells'] == 0
    assert risk['current_dominant']['false_positive_cells'] == 2
    assert risk['if_every_lane_evidence_cell_were_labeled_lane']['additional_true_positive_cells'] == 1
    assert risk['if_every_lane_evidence_cell_were_labeled_lane']['additional_false_positive_cells'] == 1
    assert risk['false_positive_cells_with_secondary_oracle_lane_evidence'] == 1
    assert risk['false_positive_cells_without_any_oracle_lane_evidence'] == 2
    assert risk['false_positive_oracle_dominant_labels']['unknown_no_oracle_cell'] == 1


def numeric():
    return dict(cell_xy=np.array([[1, 2]], dtype=np.int64), label_order=np.array([[2, 0, -1, -1, -1, -1]], dtype=np.int8),
        observed_frames=np.array([[1, 0, 1, 0, 0, 0]], dtype=np.int16),
        confidence_sum=np.array([[1., 0, 1., 0, 0, 0]], dtype=np.float64))


def test_numeric_load_keeps_label_order():
    assert study.unpack(numeric())[(1, 2)].dominant == 2


@pytest.mark.parametrize('mutation', ['order_duplicate', 'nan', 'missing_label', 'zero_positive', 'wrong_dtype', 'past240'])
def test_corrupt_grid_rejected(mutation):
    value = numeric()
    if mutation == 'order_duplicate': value['label_order'][0, 1] = 2
    elif mutation == 'nan': value['confidence_sum'][0, 0] = np.nan
    elif mutation == 'missing_label': value['label_order'][0, 0] = -1
    elif mutation == 'zero_positive': value['confidence_sum'][0, 0] = 0
    elif mutation == 'wrong_dtype': value['observed_frames'] = value['observed_frames'].astype(np.float32)
    else: value['observed_frames'][0, 0] = 241
    with pytest.raises(ValueError): study.unpack(value)


def test_actual_six_grids_exact_counts_and_original_snapshot_hashes():
    result = study.analyze()
    expected = {'ctrack_left': {'control': [175, 989, 89, 122], 'geometry': [116, 1048, 77, 134]},
                'ctrack_straight': {'control': [116, 785, 24, 247], 'geometry': [79, 801, 19, 273]}}
    for route in study.ROUTES:
        for arm in study.ARMS:
            row = result['results'][route][arm]
            assert [row['four_way'][key]['count'] for key in study.GROUPS] == expected[route][arm]
            assert row['counts_sum'] == study.EXPECTED_SUPPORT[route]
            assert row['exact_confidence_sum_tied_lane_losses']['count'] == 0
    assert result['loaded_final_numeric_grids'] == 6 and result['neural_forwards'] == 0


def test_no_model_projector_import_or_unrestricted_pickle():
    source = inspect.getsource(study)
    for forbidden in ('import torch', 'import carla', 'import portable_e2e', 'allow_pickle=True'):
        assert forbidden not in source
    assert 'np.load(path, allow_pickle=False)' in source


def test_empty_oracle_support_has_no_division_by_zero():
    result = study.partition({(0, 0): cell([0])}, {(0, 0): cell([2])})
    assert result['oracle_dominant_lane_cells'] == result['counts_sum'] == 0
    assert result['accepted_lane_evidence_missing']['fraction'] is None
