"""HH_260906 - Validate epoch weighting and reject incomplete learning-curve evidence."""

import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location('learning_curves', Path(__file__).resolve().parents[1] / 'scripts/e2e/render_portable_learning_curve.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def history():
    return [{'epoch': step // 154, 'global_step': step + 1,
        'batch_domain_sample_counts': {'carla': 1 if step % 154 == 153 else 4},
        'selected_ade_m': 10.0 if step % 154 == 153 else 1.0,
        'selected_fde_m': 20.0 if step % 154 == 153 else 2.0} for step in range(1540)]


def write_history(path, rows):
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows))


def test_actual_exposure_weighting(tmp_path):
    path = tmp_path / 'metrics.jsonl'
    write_history(path, history())
    result = MODULE.epoch_metrics(path)
    assert len(result) == 10
    assert result[-1]['epoch'] == 10
    assert result[0]['selected_ade_m'] == pytest.approx(622 / 613)
    assert result[0]['selected_fde_m'] == pytest.approx(1244 / 613)


@pytest.mark.parametrize('fault', ['missing', 'order', 'count', 'nan', 'negative', 'epoch'])
def test_bad_or_partial_history_fails(tmp_path, fault):
    rows = history()
    if fault == 'missing':
        rows.pop()
    elif fault == 'order':
        rows[1]['global_step'] = 1
    elif fault == 'count':
        rows[153]['batch_domain_sample_counts']['carla'] = 4
    elif fault == 'epoch':
        rows[0]['epoch'] = 10
    else:
        rows[0]['selected_ade_m'] = float('nan') if fault == 'nan' else -1.0
    path = tmp_path / 'metrics.jsonl'
    write_history(path, rows)
    with pytest.raises(ValueError):
        MODULE.epoch_metrics(path)
