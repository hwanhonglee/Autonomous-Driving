# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Check curve aggregation without loading weights or invoking a simulator."""
import importlib.util
from pathlib import Path

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
spec = importlib.util.spec_from_file_location('duration_curves', _hh_source_path(ROOT, 'scripts/e2e/summarize_stopmix_duration_learning.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def row(epoch, count, loss):
    return dict(epoch=epoch, batch_domain_sample_counts={'carla': count}, loss=loss)


def test_weighting_and_partial_epoch_are_explicit():
    curve = m.batch_curve([row(0, 4, 2.) for _ in range(286)] + [row(0, 3, 10.), row(1, 4, 1.)])
    assert curve[0]['sample_count'] == 1147
    assert curve[0]['complete_epoch'] is True
    assert curve[0]['sample_weighted_batch_loss'] == pytest.approx((1144 * 2 + 30) / 1147)
    assert curve[1]['complete_epoch'] is False


@pytest.mark.parametrize('count,loss', [(0, 1.), (5, 1.), (4, float('nan'))])
def test_bad_batches_fail(count, loss):
    with pytest.raises(ValueError):
        m.batch_curve([row(0, count, loss)])


def test_strict_json(tmp_path):
    path = tmp_path / 'data.json'
    path.write_text('{"a":1,"a":2}')
    with pytest.raises(ValueError, match='duplicate'):
        m.read(path)
    path.write_text('{"a":NaN}')
    with pytest.raises(ValueError, match='nonfinite'):
        m.read(path)


def test_existing_six_parents_report_no_new_training(tmp_path):
    parent = _hh_source_path(ROOT, 'artifacts/training/2026-09-14/stopmix_regret_research_v1/remote_results')
    if not parent.is_dir():
        pytest.skip('local archived research evidence unavailable')
    result = m.summarize(tmp_path / 'not_yet_run', parent, True)
    assert result['completed_epoch_runs'] == result['new_optimizer_steps'] == result['new_sample_exposures'] == 0
    assert len(result['runs']) == 6 and len(result['paired_epochs']) == 3
    assert result['status'] == 'PARTIAL_RESEARCH_NOT_PROMOTED'
    with pytest.raises(ValueError, match='missing complete epoch'):
        m.summarize(tmp_path / 'not_yet_run', parent)
