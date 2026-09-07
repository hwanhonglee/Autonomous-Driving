"""HH_260906 - Distinguish selector index concentration from geometric candidate diversity."""

import importlib.util
from pathlib import Path

import pytest
import torch

from portable_e2e.contract import ContractError


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('selector_diagnostic', ROOT / 'scripts/e2e/diagnose_portable_selector.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def predictions():
    # HH_260906 - Six parallel paths remain geometrically different even when the selector always chooses c2.
    xy = torch.zeros(2, 6, 3, 2)
    xy[:, :, :, 1] = torch.arange(6, dtype=torch.float32)[None, :, None]
    target = torch.zeros(2, 3, 2)
    target[1, :, 1] = 5
    logits = torch.zeros(2, 6)
    logits[:, 2] = 1
    return xy, logits, target, torch.ones(2, 3, dtype=torch.bool)


def test_fixed_selector_preserves_measured_geometric_diversity_and_regret():
    report = MODULE.summarize_predictions(*predictions())
    assert report['selected_histogram'] == [0, 0, 2, 0, 0, 0]
    assert report['ade_oracle_histogram'] == [1, 0, 0, 0, 0, 1]
    assert report['selector_ade_oracle_agreement_count'] == 0
    assert report['mean_selected_ade_m'] == pytest.approx(2.5)
    assert report['mean_oracle_ade_m'] == 0
    assert report['mean_selection_regret_m'] == pytest.approx(2.5)
    assert report['mean_pairwise_path_distance_m'] == pytest.approx(35 / 15)
    assert report['mean_pairwise_endpoint_distance_m'] == pytest.approx(35 / 15)


def test_identical_candidates_and_tie_policy_are_reported_without_false_diversity():
    xy, logits, target, valid = predictions()
    xy.zero_()
    target.zero_()
    logits.zero_()
    report = MODULE.summarize_predictions(xy, logits, target, valid)
    assert report['selected_histogram'] == report['ade_oracle_histogram'] == [2, 0, 0, 0, 0, 0]
    assert report['selector_ade_oracle_agreement_rate'] == 1
    assert report['mean_selection_regret_m'] == report['mean_pairwise_path_distance_m'] == 0


def test_ade_ignores_invalid_target_points_but_geometry_keeps_full_predictions():
    xy, logits, target, valid = predictions()
    valid[:, -1] = False
    target[:, -1] = 10000
    report = MODULE.summarize_predictions(xy, logits, target, valid)
    assert report['mean_selected_ade_m'] == pytest.approx(2.5)
    assert report['mean_pairwise_path_distance_m'] == pytest.approx(35 / 15)


@pytest.mark.parametrize('fault', ['nan', 'empty_mask', 'integer_mask', 'bad_logits', 'wrong_candidates'])
def test_malformed_predictions_rejected(fault):
    xy, logits, target, valid = predictions()
    if fault == 'nan':
        xy[0, 0, 0, 0] = float('nan')
    elif fault == 'empty_mask':
        valid[0] = False
    elif fault == 'integer_mask':
        valid = valid.long()
    elif fault == 'bad_logits':
        logits = logits[:, :5]
    else:
        xy = xy[:, :5]
    with pytest.raises(ContractError):
        MODULE.summarize_predictions(xy, logits, target, valid)


def test_render_positions_are_frozen_and_full_val_is_required():
    assert MODULE.phase_indices(337) == [0, 67, 134, 201, 268, 336]
    with pytest.raises(ContractError):
        MODULE.phase_indices(6)


def test_cli_requires_checkpoint_hash_and_has_no_test_or_gpu_switch():
    with pytest.raises(SystemExit):
        MODULE.parse_args(['dataset', '--checkpoint', 'model.pt', '--output-dir', 'output'])
    for forbidden in ('--split', '--device'):
        with pytest.raises(SystemExit):
            MODULE.parse_args(['dataset', '--checkpoint', 'model.pt', '--checkpoint-sha256', 'a' * 64,
                '--output-dir', 'output', forbidden, 'test' if forbidden == '--split' else 'cuda:0'])
