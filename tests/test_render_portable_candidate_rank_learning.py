"""HH_260906 - Verify paired raw-input binding and full-plus-partial epoch exposure accounting."""

import copy
import importlib.util
import json
from pathlib import Path

import pytest

from portable_e2e.contract import ContractError

SPEC = importlib.util.spec_from_file_location('candidate_rank_learning',
    Path(__file__).resolve().parents[1] / 'scripts/e2e/render_portable_candidate_rank_learning.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def history():
    rows, seen = [], 0
    for offset in range(1540):
        count = 3 if (offset + 1) % 287 == 0 else 4
        seen += count
        rows.append({'global_step': offset + 1, 'epoch': offset // 287,
            'batch_domain_sample_counts': {'carla': count}, 'samples_seen': seen,
            'domain_samples_seen': {'carla': seen},
            **{metric: 10.0 if count == 3 else 1.0 for metric in MODULE.TRAIN_METRICS}})
    return rows


def payload(rows):
    return ''.join(json.dumps(row) + '\n' for row in rows).encode()


def test_all_measured_steps_and_partial_sixth_epoch_are_weighted_correctly():
    rows, epochs = MODULE.training_history(payload(history()))
    assert len(rows) == 1540
    assert len(epochs) == 6
    assert [row['sample_exposures'] for row in epochs] == [1147] * 5 + [420]
    assert [row['batch_count'] for row in epochs] == [287] * 5 + [105]
    assert sum(row['sample_exposures'] for row in epochs) == 6155
    assert epochs[0]['selected_ade_m'] == pytest.approx((1144 + 30) / 1147)
    assert epochs[-1]['selected_ade_m'] == pytest.approx(1)
    assert epochs[-1]['complete'] is False and 'PARTIAL' in epochs[-1]['label']
    assert epochs[-1]['first_step'] == 1436 and epochs[-1]['last_step'] == 1540


@pytest.mark.parametrize('fault', ['missing', 'extra', 'step', 'step_bool', 'epoch', 'epoch_float',
    'wrong_tail_count', 'negative_domain', 'mixed_domain', 'float_batch', 'counter', 'float_counter',
    'domain_counter', 'nan', 'inf', 'bool_metric', 'string_metric', 'negative_metric', 'huge_integer', 'missing_metric'])
def test_bad_history_fails_closed(fault):
    rows = history()
    if fault == 'missing': rows.pop()
    elif fault == 'extra': rows.append(rows[-1])
    elif fault == 'step': rows[1]['global_step'] = 1
    elif fault == 'step_bool': rows[0]['global_step'] = True
    elif fault == 'epoch': rows[287]['epoch'] = 0
    elif fault == 'epoch_float': rows[0]['epoch'] = 0.0
    elif fault == 'wrong_tail_count': rows[286]['batch_domain_sample_counts']['carla'] = 4
    elif fault == 'negative_domain': rows[0]['batch_domain_sample_counts'] = {'carla': 5, 'real': -1}
    elif fault == 'mixed_domain': rows[0]['batch_domain_sample_counts'] = {'carla': 3, 'real': 1}
    elif fault == 'float_batch': rows[0]['batch_domain_sample_counts']['carla'] = 4.0
    elif fault == 'counter': rows[-1]['samples_seen'] = 6160
    elif fault == 'float_counter': rows[0]['samples_seen'] = 4.0
    elif fault == 'domain_counter': rows[0]['domain_samples_seen']['carla'] = 0
    elif fault == 'missing_metric': del rows[0]['regression_loss']
    else:
        rows[0]['candidate_score_loss'] = {'nan': float('nan'), 'inf': float('inf'),
            'bool_metric': True, 'string_metric': '1', 'negative_metric': -1, 'huge_integer': 10 ** 400}[fault]
    with pytest.raises(ContractError):
        MODULE.training_history(payload(rows))


def test_duplicate_json_keys_and_blank_rows_are_rejected():
    value = payload(history())
    for bad in (value.replace(b'"epoch": 0', b'"epoch": 0, "epoch": 0', 1), b'\n' + value):
        with pytest.raises(ContractError):
            MODULE.training_history(bad)


def summary():
    return {'schema': MODULE.campaign_summary.SCHEMA, 'status': 'COMPLETE_NOT_PROMOTED',
        'automatic_promotion': False, 'vehicle_control_approved': False, 'test_opened_by_this_campaign': False,
        'candidate_screen': 'FAIL', 'absolute_quality': 'FAIL',
        'pairs': [{'seed': seed, **{label: {'arm': arm, 'training_dataset_size': 1147,
            'validation_sample_count': 337, 'metrics': {metric: 2.5 + index for index, metric in enumerate(MODULE.VAL_METRICS)}}
            for label, arm in MODULE.ARMS.items()}} for seed in MODULE.SEEDS]}


@pytest.fixture
def inputs(tmp_path, monkeypatch):
    roots = {label: tmp_path / label for label in MODULE.ARMS}
    for seed in MODULE.SEEDS:
        for label, arm in MODULE.ARMS.items():
            path = roots[label] / f'seed_{seed}/{arm}/training/metrics.jsonl'
            path.parent.mkdir(parents=True)
            path.write_bytes(payload(history()))
    report = summary()
    path = tmp_path / 'summary.json'
    path.write_text(json.dumps(report))
    calls = []

    def fresh(baseline, candidate):
        calls.append((baseline, candidate))
        return copy.deepcopy(report)

    monkeypatch.setattr(MODULE.campaign_summary, 'summarize_campaign', fresh)
    return roots, path, report, calls


def fake_draw(output, report, measurements):
    for name in ('01_training_curves.png', '02_paired_validation.png'):
        (output / name).write_bytes(b'unit test figure placeholder')


def test_strict_summary_recomputed_and_six_history_paths_bound(inputs, tmp_path, monkeypatch):
    roots, path, report, calls = inputs
    monkeypatch.setattr(MODULE, 'draw_plots', fake_draw)
    output = tmp_path / 'plots'
    result = MODULE.render(roots['baseline'], roots['candidate'], path, output)
    assert calls == [(roots['baseline'], roots['candidate'])] * 2
    assert result['input_layout'] == 'paired_campaign_roots_v1'
    assert result['summary_sha256'] == MODULE.sha256(path.read_bytes())
    assert len(result['inputs']) == 6
    for item in result['inputs']:
        label, relative = item['path'].split('/', 1)
        assert item['sha256'] == MODULE.sha256((roots[label] / relative).read_bytes())
    assert all(item['complete_epochs'] == 5 and item['partial_epoch_exposures'] == 420
        for item in result['training_epoch_metrics'])
    assert (output / 'plot_inputs.json').is_file()
    with pytest.raises(ContractError, match='must be new'):
        MODULE.render(roots['baseline'], roots['candidate'], path, output)


@pytest.mark.parametrize('fault', ['stale_summary', 'incomplete', 'wrong_arm', 'wrong_seed', 'nonfinite_val', 'bad_history'])
def test_unverified_or_incomplete_inputs_create_no_output(inputs, tmp_path, monkeypatch, fault):
    roots, path, report, _ = inputs
    if fault == 'stale_summary':
        stale = copy.deepcopy(report); stale['candidate_screen'] = 'PASS'
        path.write_text(json.dumps(stale))
    elif fault == 'bad_history':
        rows = history(); rows.pop()
        (roots['candidate'] / 'seed_20260903/E_candidate_rank/training/metrics.jsonl').write_bytes(payload(rows))
    else:
        if fault == 'incomplete': report['status'] = 'INCOMPLETE'
        elif fault == 'wrong_arm': report['pairs'][0]['candidate']['arm'] = '../other'
        elif fault == 'wrong_seed': report['pairs'][0]['seed'] = 20260905
        else: report['pairs'][0]['candidate']['metrics']['selected_ade_m'] = True
        path.write_text(json.dumps(report))
    output = tmp_path / 'plots'
    with pytest.raises(ContractError):
        MODULE.render(roots['baseline'], roots['candidate'], path, output)
    assert not output.exists()


@pytest.mark.parametrize('changed', ['history', 'summary'])
def test_inputs_changed_while_drawing_never_get_publication_manifest(inputs, tmp_path, monkeypatch, changed):
    roots, path, report, _ = inputs

    def mutate(output, report, measurements):
        fake_draw(output, report, measurements)
        if changed == 'summary':
            path.write_text(path.read_text() + '\n')
        else:
            raw = roots['baseline'] / 'seed_20260903/C_expanded_data/training/metrics.jsonl'
            raw.write_bytes(raw.read_bytes() + b'\n')

    monkeypatch.setattr(MODULE, 'draw_plots', mutate)
    output = tmp_path / 'plots'
    with pytest.raises(ContractError, match='changed'):
        MODULE.render(roots['baseline'], roots['candidate'], path, output)
    assert not (output / 'plot_inputs.json').exists()


def test_actual_figures_are_1920_pixels_wide_and_not_empty(inputs, tmp_path):
    pytest.importorskip('matplotlib')
    image = pytest.importorskip('PIL.Image')
    roots, path, _, _ = inputs
    output = tmp_path / 'actual_figures'
    result = MODULE.render(roots['baseline'], roots['candidate'], path, output)
    for item in result['plots']:
        with image.open(output / item['path']) as png:
            assert png.width == 1920
            assert png.height in (1200, 1320)
            assert png.getbbox() is not None
        assert (output / item['path']).stat().st_size > 20000
