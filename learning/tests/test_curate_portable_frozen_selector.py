"""HH_260906 - Verify create-only publication, exact route bytes, metadata privacy and measured plot provenance."""

import json
from pathlib import Path
import struct

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import curate_portable_frozen_selector as curator
from test_summarize_portable_frozen_selector import campaign, read_json, reseal, write_json


@pytest.fixture
def evidence(campaign, tmp_path, monkeypatch):
    # HH_260906 - Reuse the strict nine-fit fixture instead of bypassing source, batch or artifact validation.
    root, parent = campaign
    summary_path = tmp_path / 'verified/summary.json'
    report = curator.strict.summarize_campaign(root, parent)
    write_json(summary_path, report)
    state = read_json(root / 'status.json')
    state['completed_at_utc'] = '2026-09-08T00:02:02+00:00'
    write_json(root / 'status.json', state)
    write_json(summary_path, curator.strict.summarize_campaign(root, parent))

    def fake_draw(output, report, measured):
        for name in curator.PLOTS:
            (output / name).write_bytes(b'synthetic rendered PNG')

    monkeypatch.setattr(curator, 'draw_plots', fake_draw)
    return root, parent, summary_path, tmp_path / 'public'


def test_publish_exact_72_routes_and_separate_metadata_views(evidence):
    root, parent, summary_path, output = evidence
    result = curator.curate(*evidence)
    assert result['status'] == 'PUBLISHED_NOT_PROMOTED' and result['original_route_png_count'] == 72
    assert len(list((output / 'routes').rglob('*.png'))) == 72
    assert not list(output.rglob('*.pt')) and not list(output.rglob('*.py'))
    manifest = read_json(output / 'publication_manifest.json')
    for item in manifest['files']:
        public = output / item['published_path']
        assert curator.sha(public) == item['sha256']
        if item['representation'] == 'original':
            assert item['raw_source_sha256'] == item['sha256']
            assert public.read_bytes() == (root / item['source_path'].removeprefix('campaign/')).read_bytes()
    public_head = read_json(output / 'runs/seed_20260903/linear_continue/report.json')
    assert public_head['omitted_fields'] == list(curator.OMITTED_HEAD_FIELDS)
    assert all(field not in public_head for field in curator.OMITTED_HEAD_FIELDS)
    assert public_head['raw_source_sha256'] == curator.sha(root / 'seed_20260903/linear_continue/report.json')
    assert read_json(output / 'summary.json')['raw_source_sha256'] == curator.sha(summary_path)
    entries = (output / 'SHA256SUMS').read_text().splitlines()
    assert len(entries) == len([path for path in output.rglob('*') if path.is_file()]) - 1
    for entry in entries:
        digest, path = entry.split('  ', 1)
        assert curator.sha(output / path) == digest
    assert len(list((output / 'routes').rglob('README.md'))) == 12


def test_no_overwrite_or_raw_input_mutation(evidence):
    root, parent, summary_path, output = evidence
    originals = {path: curator.sha(path) for path in root.rglob('*') if path.is_file()}
    curator.curate(*evidence)
    before = curator.sha(output / 'publication_manifest.json')
    with pytest.raises(ContractError, match='new directory'):
        curator.curate(*evidence)
    assert curator.sha(output / 'publication_manifest.json') == before
    assert originals == {path: curator.sha(path) for path in originals}
    with pytest.raises(ContractError, match='read-only input'):
        curator.curate(root, parent, summary_path, root / 'bad_publication')


@pytest.mark.parametrize('mutation', ['summary', 'history', 'parent', 'route', 'runner', 'plan'])
def test_stale_or_corrupted_evidence_rejected_before_output(evidence, mutation):
    root, parent, summary_path, output = evidence
    if mutation == 'summary':
        report = read_json(summary_path)
        report['arm_screens']['linear_continue']['candidate_screen'] = 'UNVERIFIED'
        write_json(summary_path, report)
    else:
        path = {'history': root / 'seed_20260903/linear_continue/metrics.jsonl',
            'parent': parent / 'seed_20260903/C_expanded_data/evaluation/metrics.json',
            'route': root / 'seed_20260903/original_c_routes/val_phase_000.png',
            'runner': root / 'provenance/active_runner.py', 'plan': root / 'provenance/plan.json'}[mutation]
        path.write_bytes(b'corrupted')
    with pytest.raises((ContractError, ValueError)):
        curator.curate(*evidence)
    assert not output.exists()


def test_numeric_epochs_use_sample_weights_and_preserve_partial_label(evidence):
    root, parent, summary_path, output = evidence
    measured = curator.measurements(root, read_json(summary_path))
    assert len(measured) == 9
    for item in measured:
        assert len(item['rows']) == 1540
        assert [epoch['sample_exposures'] for epoch in item['epochs']] == [1147] * 5 + [420]
        assert [epoch['complete'] for epoch in item['epochs']] == [True] * 5 + [False]
        assert all(epoch['candidate_score_loss'] == pytest.approx(2.0) for epoch in item['epochs'])
        assert item['path'].startswith('campaign/seed_')
    curator.curate(*evidence)
    plots = read_json(output / 'visuals/plot_inputs.json')
    assert plots['summary_raw_sha256'] == curator.sha(summary_path)
    assert len(plots['inputs']) == 9 and all('rows' not in item for item in plots['inputs'])
    assert len(plots['publication_code_sha256']) == 4


def test_account_paths_redacted_but_raw_sha_retained(evidence):
    root, parent, summary_path, output = evidence
    path = root / 'seed_20260903/linear_continue/report.json'
    head = read_json(path)
    head['interpretation'] = '/home/other-user/personal/another-owner/portable_e2e/runs/private'
    write_json(path, head)
    reseal(root)
    write_json(summary_path, curator.strict.summarize_campaign(root, parent))
    curator.curate(*evidence)
    public = read_json(output / 'runs/seed_20260903/linear_continue/report.json')
    assert '/home/' not in json.dumps(public)
    assert '${PORTABLE_E2E_ROOT}/runs/private' in public['interpretation']
    assert public['raw_source_sha256'] == curator.sha(path)


def test_change_during_rendering_never_issues_success_manifest(evidence, monkeypatch):
    root, parent, summary_path, output = evidence

    def changing_draw(directory, report, measured):
        for name in curator.PLOTS:
            (directory / name).write_bytes(b'plot')
        (root / 'seed_20260903/train_cache.pt').write_bytes(b'changed while plotting')

    monkeypatch.setattr(curator, 'draw_plots', changing_draw)
    with pytest.raises(ContractError):
        curator.curate(*evidence)
    assert not (output / 'publication_manifest.json').exists()
    assert not (output / 'SHA256SUMS').exists()


def test_actual_plot_dimensions_and_no_new_validation_points(campaign, tmp_path):
    root, parent = campaign
    report = curator.strict.summarize_campaign(root, parent)
    measured = curator.measurements(root, report)
    output = tmp_path / 'plots'
    output.mkdir()
    curator.draw_plots(output, report, measured)
    for name in curator.PLOTS:
        payload = (output / name).read_bytes()
        assert payload[:8] == b'\x89PNG\r\n\x1a\n'
        assert struct.unpack('>II', payload[16:24])[0] == 1920
    assert all(len(item['epochs']) == 6 for item in measured)
