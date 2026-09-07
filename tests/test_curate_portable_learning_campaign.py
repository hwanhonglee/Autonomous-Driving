"""HH_260906 - Keep learning evidence publication create-only and checkpoint-free."""

import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location('curate_learning', Path(__file__).resolve().parents[1] / 'scripts/e2e/curate_portable_learning_campaign.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def setup_evidence(tmp_path, monkeypatch, schema='portable_e2e.lr_ab_campaign.v1'):
    raw, summary, output = (tmp_path / name for name in ('raw', 'summary', 'published'))
    raw.mkdir()
    summary.mkdir()
    frozen = raw / 'provenance/active_runner.py'
    frozen.parent.mkdir()
    frozen.write_text('# HH_260906 - Frozen test runner.\n')
    run = 'seed_20260903/A_baseline'
    for name in ('training/run.json', 'training/metrics.jsonl', 'evaluation/metrics.json', 'gate_v8.json', 'training/checkpoints/latest.pt'):
        path = raw / run / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('{}\n')
    state = {'status': 'TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED', 'runner_sha256': MODULE.sha(frozen), 'source_commit': 'a' * 40,
        'stages': [{'run': run}]}
    (raw / 'status.json').write_text(json.dumps(state))
    (raw / 'plan.json').write_text(json.dumps({'schema': schema}))

    def fresh_report(root):
        return {'status': 'COMPLETE_NOT_PROMOTED', 'vehicle_control_approved': False,
            'input_manifest': [{'path': f'{run}/evaluation/metrics.json',
                'sha256': MODULE.sha(root / run / 'evaluation/metrics.json')}]}

    summarizer = MODULE.training_summary if schema == 'portable_e2e.lr_ab_campaign.v1' else MODULE.expansion_summary
    monkeypatch.setattr(summarizer, 'summarize_campaign', fresh_report)
    monkeypatch.setattr(summarizer, 'render_markdown', lambda report: 'Test evidence\n')
    (summary / 'summary.json').write_text(json.dumps(fresh_report(raw)))
    (summary / 'README.md').write_text('Test evidence\n')
    return raw, summary, output


@pytest.mark.parametrize('schema,summarizer', [
    ('portable_e2e.selector_weight_campaign.v1', MODULE.selector_summary),
    ('portable_e2e.candidate_rank_campaign.v1', MODULE.ranking_summary),
])
def test_selector_recomputes_both_campaigns_and_requires_baseline(tmp_path, monkeypatch, schema, summarizer):
    # HH_260906 - A paired summary must bind the baseline as well as the candidate campaign.
    candidate, baseline = tmp_path / 'candidate', tmp_path / 'baseline'
    seen = []
    report = {'status': 'COMPLETE_NOT_PROMOTED'}
    monkeypatch.setattr(summarizer, 'summarize_campaign',
        lambda before, after: seen.append((before, after)) or report)
    monkeypatch.setattr(summarizer, 'render_markdown', lambda value: 'paired\n')
    plan = {'schema': schema}
    with pytest.raises(ValueError, match='baseline'):
        MODULE.recompute_summary(candidate, plan)
    assert MODULE.recompute_summary(candidate, plan, baseline) == (report, 'paired\n')
    assert seen == [(baseline, candidate)]


def test_unrelated_baseline_is_not_silently_ignored(tmp_path):
    with pytest.raises(ValueError, match='only for selector'):
        MODULE.recompute_summary(tmp_path, {'schema': 'portable_e2e.lr_ab_campaign.v1'}, tmp_path / 'baseline')


@pytest.mark.parametrize('mutation', [None, 'missing_baseline', 'wrong_hash', 'wrong_layout', 'escape', 'duplicate'])
def test_paired_plot_inputs_bind_both_campaign_roots(tmp_path, mutation):
    # HH_260906 - A paired architecture plot cannot silently omit or substitute the baseline histories.
    candidate, baseline, plots = [tmp_path / name for name in ('candidate', 'baseline', 'plots')]
    plots.mkdir()
    state = {'stages': [{'run': 'seed_20260903/E_candidate_rank'}]}
    before = {'stages': [{'run': 'seed_20260903/C_expanded_data'}]}
    inputs = []
    for name, root, entry in (('candidate', candidate, state), ('baseline', baseline, before)):
        path = root / entry['stages'][0]['run'] / 'training/metrics.jsonl'
        path.parent.mkdir(parents=True)
        path.write_text('{}\n')
        (root / 'status.json').write_text(json.dumps(entry))
        inputs.append({'path': f'{name}/{path.relative_to(root)}', 'sha256': MODULE.sha(path)})
    summary = tmp_path / 'summary.json'
    summary.write_text('{}')
    payload = {'input_layout': 'paired_campaign_roots_v1', 'summary_sha256': MODULE.sha(summary), 'inputs': inputs}
    if mutation == 'missing_baseline':
        inputs.pop()
    elif mutation == 'wrong_hash':
        inputs[-1]['sha256'] = 'f' * 64
    elif mutation == 'wrong_layout':
        payload.pop('input_layout')
    elif mutation == 'escape':
        inputs[-1]['path'] = 'baseline/../other/training/metrics.jsonl'
    elif mutation == 'duplicate':
        inputs[-1] = dict(inputs[0])
    (plots / 'plot_inputs.json').write_text(json.dumps(payload))
    if mutation is None:
        MODULE.verify_plots(plots, candidate, summary, state, baseline)
        with pytest.raises(ValueError, match='baseline'):
            MODULE.verify_plots(plots, candidate, summary, state)
    else:
        with pytest.raises(ValueError):
            MODULE.verify_plots(plots, candidate, summary, state, baseline)


@pytest.mark.parametrize('schema', ['portable_e2e.lr_ab_campaign.v1', 'portable_e2e.data_expansion_campaign.v1'])
def test_metadata_views_no_checkpoint_or_runner_and_no_overwrite(tmp_path, monkeypatch, schema):
    raw, summary, output = setup_evidence(tmp_path, monkeypatch, schema)
    args = [str(raw), '--summary-dir', str(summary), '--output', str(output)]
    assert MODULE.main(args) == 0
    assert not list(output.rglob('*.pt'))
    assert not list(output.rglob('*.py'))
    public = json.loads((output / 'summary.json').read_text())
    assert public['status'] == 'COMPLETE_NOT_PROMOTED'
    assert public['raw_source_sha256'] == MODULE.sha(summary / 'summary.json')
    assert 'private original' in public['publication_notice']
    private_runner = raw / 'provenance/active_runner.py'
    execution = json.loads((output / 'provenance/execution_record.json').read_text())
    assert execution['runner_sha256'] == MODULE.sha(private_runner)
    assert (output / 'runs/seed_20260903/A_baseline/training/metrics.jsonl').read_bytes() == (raw / 'seed_20260903/A_baseline/training/metrics.jsonl').read_bytes()
    with pytest.raises(FileExistsError):
        MODULE.main(args)


def test_runner_tampering_stops_before_publication(tmp_path, monkeypatch):
    raw, summary, output = setup_evidence(tmp_path, monkeypatch)
    (raw / 'provenance/active_runner.py').write_text('changed')
    with pytest.raises(ValueError, match='runner'):
        MODULE.main([str(raw), '--summary-dir', str(summary), '--output', str(output)])
    assert not output.exists()


def test_incomplete_summary_is_not_published(tmp_path, monkeypatch):
    raw, summary, output = setup_evidence(tmp_path, monkeypatch)
    (summary / 'summary.json').write_text(json.dumps({'status': 'INCOMPLETE'}))
    with pytest.raises(ValueError, match='completed'):
        MODULE.main([str(raw), '--summary-dir', str(summary), '--output', str(output)])
    assert not output.exists()


@pytest.mark.parametrize('change', ['summary', 'metric', 'readme', 'schema'])
def test_stale_or_unbound_summary_rejected_before_output(tmp_path, monkeypatch, change):
    raw, summary, output = setup_evidence(tmp_path, monkeypatch)
    if change == 'summary':
        report = json.loads((summary / 'summary.json').read_text())
        report['candidate_screen'] = 'PASS'
        (summary / 'summary.json').write_text(json.dumps(report))
    elif change == 'metric':
        (raw / 'seed_20260903/A_baseline/evaluation/metrics.json').write_text('{"changed":true}')
    elif change == 'readme':
        (summary / 'README.md').write_text('False PASS from another summary')
    else:
        (raw / 'plan.json').write_text('{"schema":"unreviewed"}')
    with pytest.raises(ValueError, match='summary|README|schema'):
        MODULE.main([str(raw), '--summary-dir', str(summary), '--output', str(output)])
    assert not output.exists()


def setup_plots(tmp_path, raw, summary):
    plots = tmp_path / 'plots'
    plots.mkdir()
    relative = 'seed_20260903/A_baseline/training/metrics.jsonl'
    manifest = {'summary_sha256': MODULE.sha(summary / 'summary.json'),
        'inputs': [{'path': relative, 'sha256': MODULE.sha(raw / relative)}]}
    (plots / 'plot_inputs.json').write_text(json.dumps(manifest))
    (plots / 'figure.png').write_bytes(b'unchanged PNG fixture bytes')
    return plots, manifest


@pytest.mark.parametrize('change', ['summary_hash', 'input_hash', 'input_path', 'missing_input', 'duplicate_input'])
def test_plot_provenance_mismatch_rejected_before_output(tmp_path, monkeypatch, change):
    raw, summary, output = setup_evidence(tmp_path, monkeypatch)
    plots, manifest = setup_plots(tmp_path, raw, summary)
    if change == 'summary_hash':
        manifest['summary_sha256'] = 'b' * 64
    elif change == 'input_hash':
        manifest['inputs'][0]['sha256'] = 'b' * 64
    elif change == 'input_path':
        manifest['inputs'][0]['path'] = '../outside/metrics.jsonl'
    elif change == 'missing_input':
        manifest['inputs'] = []
    else:
        manifest['inputs'] *= 2
    (plots / 'plot_inputs.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='plot'):
        MODULE.main([str(raw), '--summary-dir', str(summary), '--plots-dir', str(plots), '--output', str(output)])
    assert not output.exists()


def test_redacted_views_preserve_original_hashes_and_png_bytes(tmp_path, monkeypatch):
    raw, summary, output = setup_evidence(tmp_path, monkeypatch)
    state = json.loads((raw / 'status.json').read_text())
    state['example_paths'] = [
        '/home/test-remote/personal/another-owner/portable_e2e/runs/model.pt',
        '/home/test-remote/personal/another-owner/dataset/prepared',
        '/home/test-local/autoware_e2e/artifacts', '/tmp/example/output',
    ]
    (raw / 'status.json').write_text(json.dumps(state))
    plots, _ = setup_plots(tmp_path, raw, summary)
    assert MODULE.main([str(raw), '--summary-dir', str(summary), '--plots-dir', str(plots), '--output', str(output)]) == 0
    published = json.loads((output / 'provenance/status.json').read_text())
    assert published['example_paths'] == ['${PORTABLE_E2E_ROOT}/runs/model.pt',
        '${PERSONAL_DATASET_ROOT}/prepared', '${REPO_ROOT}/artifacts', '${TMP_ROOT}/example/output']
    assert published['raw_source_sha256'] == MODULE.sha(raw / 'status.json')
    assert (raw / 'status.json').read_text().find('/home/test-remote') >= 0
    assert (output / 'visuals/figure.png').read_bytes() == (plots / 'figure.png').read_bytes()
    manifest = json.loads((output / 'publication_manifest.json').read_text())
    for entry in manifest['files']:
        assert entry['sha256'] == MODULE.sha(output / entry['published_path'])
        assert '/home/test-' not in entry['source_path']
        assert len(entry['raw_source_sha256']) == 64
