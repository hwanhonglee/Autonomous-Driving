"""HH_260906 - CPU-only checks for exact DEV qualification; no actual capture is converted here."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_carla_common10_dev_v2 as adapter
from scripts.e2e import qualify_carla_moving_dev_common10_v2 as qualification

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


@pytest.mark.parametrize('value', [None, '', 'a' * 63, 'g' * 64, 'A' * 64, 123])
def test_external_sha_required(value):
    with pytest.raises(adapter.AdapterError):
        qualification.hash_value(value)


@pytest.mark.parametrize('container', [(), ('result',), ('capture_contract',),
    ('capture_contract', 'agent_initialization'), ('capture_contract', 'goal_stop_profile'),
    ('result', 'goal_stop_quality', 'development_pilot')])
@pytest.mark.parametrize('marker,value', [('training_data_approved', False), ('development_only', True),
    ('training_data_approved', 'false')])
def test_explicit_denials_cannot_be_overridden(container, marker, value):
    manifest = {}; nested = manifest
    for key in container:
        nested = nested.setdefault(key, {})
    nested[marker] = value
    with pytest.raises((adapter.AdapterError, qualification.contract.ContractError)):
        qualification.check_denials(manifest)


def test_wrong_receipt_pin_rejects_before_metadata_or_payload(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    monkeypatch.setattr(qualification, 'document', lambda *_: pytest.fail('opened metadata after bad external pin'))
    with pytest.raises(adapter.AdapterError, match='external SHA'):
        qualification.authorize(SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256='a' * 64))


def test_changed_jpeg_and_matching_rewritten_journal_do_not_requalify(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    audit = tmp_path / 'summary.json'; audit.write_text('{}')
    image = tmp_path / 'image.jpg'; image.write_bytes(b'original')
    journal = tmp_path / 'jpeg_audit.jsonl'
    journal.write_text(json.dumps({'path': image.name, 'sha256': qualification.sha(image)}))
    monkeypatch.setattr(qualification, 'COMPANION_SHA', {'jpeg_audit.jsonl': qualification.sha(journal)})
    qualification.companion_pins(audit)
    image.write_bytes(b'altered image')
    journal.write_text(json.dumps({'path': image.name, 'sha256': qualification.sha(image)}))
    with pytest.raises(adapter.AdapterError, match='historical diagnostic companion'):
        qualification.companion_pins(audit)


def test_symlinks_and_escaping_paths_rejected(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    real = tmp_path / 'real'; real.mkdir()
    link = tmp_path / 'link'; link.symlink_to(real, target_is_directory=True)
    for path in (link / 'data', tmp_path / '..' / 'escape', tmp_path.parent / 'foreign'):
        with pytest.raises(adapter.AdapterError):
            qualification.safe(path)


def test_denial_stops_native_load_before_rows(tmp_path, monkeypatch):
    path = tmp_path / 'native'; path.mkdir()
    for name in ('manifest.json', 'route.json', 'states.jsonl', 'camera_frames.jsonl'):
        (path / name).write_text('{}')
    (path / 'manifest.json').write_text(json.dumps({'capture_contract': {'agent_initialization': {'training_data_approved': False}}}))
    monkeypatch.setattr(adapter.base, '_jsonl', lambda *_: pytest.fail('denied native rows opened'))
    with pytest.raises(adapter.AdapterError, match='prohibits'):
        adapter._load_native(adapter.EpisodeInput('val', path), moving_qualification={})


@pytest.mark.parametrize('split', ['train', 'test'])
def test_authorizer_rejects_non_val_before_native_payload(tmp_path, monkeypatch, split):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256=qualification.sha(receipt),
        episode=[adapter.EpisodeInput(split, tmp_path / 'native')])
    monkeypatch.setattr(qualification, 'document', lambda *_: pytest.fail('non-VAL metadata opened'))
    with pytest.raises(adapter.AdapterError, match='VAL input'):
        qualification.authorize(args)


@pytest.mark.parametrize('mode', ['hardlink', 'runtime'])
def test_authorizer_requires_copy_planning(tmp_path, monkeypatch, mode):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    monkeypatch.setattr(qualification, 'CASE', 'case')
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256=qualification.sha(receipt),
        episode=[adapter.EpisodeInput('val', tmp_path / 'case/episode')],
        image_mode='hardlink' if mode == 'hardlink' else 'copy', validate_mode='runtime' if mode == 'runtime' else 'planning')
    with pytest.raises(adapter.AdapterError, match='copy/planning'):
        qualification.authorize(args)


def test_recheck_rejects_changed_input_and_source(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    source = tmp_path / 'source.py'; source.write_text('original')
    item = dict(receipt_path=str(receipt), receipt_sha256=qualification.sha(receipt),
        document={'input_sha256': {}, 'source_sha256': {'source.py': qualification.sha(source)}})
    qualification.recheck(item)
    source.write_text('changed')
    with pytest.raises(adapter.AdapterError, match='source/input changed'):
        qualification.recheck(item)


@pytest.mark.parametrize('payload', ['{"x":NaN}', '{"x":1e999}', '{"x":0,"x":1}'])
def test_strict_json(tmp_path, monkeypatch, payload):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    path = tmp_path / 'bad.json'; path.write_text(payload)
    with pytest.raises((ValueError, adapter.AdapterError)):
        qualification.document(path)


def test_receipt_cli_never_overwrites_existing_path(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    output = tmp_path / 'receipt.json'; output.write_bytes(b'untouched')
    monkeypatch.setattr(qualification, 'evidence', lambda *_args, **_kwargs: pytest.fail('read evidence after existing output'))
    with pytest.raises(adapter.AdapterError, match='fresh'):
        qualification.main(['--case-root', str(tmp_path / 'case'), '--raw-audit', str(tmp_path / 'analysis/summary.json'),
            '--raw-audit-sha256', 'a' * 64, '--output', str(output)])
    assert output.read_bytes() == b'untouched'


@pytest.fixture
def captured_dev():
    # HH_260906 - The local immutable capture supplies metadata only; no simulator, conversion or receipt writes occur.
    case = _hh_source_path(qualification.ROOT, qualification.CASE)
    if not (case / 'episode/manifest.json').is_file():
        pytest.skip('local immutable DEV capture is unavailable')
    manifest = qualification.document(case / 'episode/manifest.json')
    return case, manifest


def test_actual_native_metadata_and_selected_reconstruction(captured_dev):
    from collections import Counter
    case, manifest = captured_dev
    qualification.scope_manifest(manifest)
    states, cameras, selected = qualification.selected_replay(case, manifest)
    assert len(states) == 1151 and Counter(row['capture_phase'] for row in states) == dict(stationary_warmup=70, driving=951, stationary_tail=130)
    assert len(cameras) == 576 and len(selected) == 328 and len(cameras) - len(selected) == 248
    assert [row['frame'] for row in selected] == list(range(1261, 1916, 2))
    assert selected[-1]['timestamp_ns'] - selected[0]['timestamp_ns'] == 32_700_000_487
    owner = qualification.document(case / 'owner_plan.json')
    route = qualification.document(case / 'episode/route.json')
    identity = qualification.direct_route_identity(case, manifest, route, owner)
    assert identity['route_id'] == 'town03_straight_s0000_p07'
    assert identity['coordinate_alignment_synthesized'] is False
    assert qualification.sha(case / 'owner_plan.json') == qualification.OWNER_SHA
    assert qualification.sha(case / 'episode/moving_task_plan.json') == qualification.PLAN_SHA


@pytest.mark.parametrize('mutation', ['town', 'schema', 'split', 'route_selection', 'legacy_plan'])
def test_native_dev_scope_rejects_other_contracts(captured_dev, mutation):
    _case, manifest = captured_dev
    manifest = copy.deepcopy(manifest)
    moving = manifest['capture_contract']['moving_task']
    if mutation == 'town':
        moving['town'] = 'Town07'
    elif mutation == 'schema':
        moving['schema'] = 'carla.moving_task_capture_plan.v1'
    elif mutation == 'split':
        moving['dataset_split_intent'] = 'train'
    elif mutation == 'route_selection':
        moving['route_selection_evidence']['sha256'] = '0' * 64
    else:
        moving['plan_sha256'] = 'f48132acb3e4da42767384d2e18b2ff04d2f719288c01071ef04ce7c17d23877'
    with pytest.raises(adapter.AdapterError):
        qualification.scope_manifest(manifest)


@pytest.mark.parametrize('split', ['train', 'test'])
def test_verified_native_scope_refuses_non_val(captured_dev, split):
    case, manifest = captured_dev
    with pytest.raises(adapter.AdapterError, match='wrong moving native input'):
        qualification.verify_native_scope(adapter.EpisodeInput(split, case / 'episode'), manifest, {})


def test_legacy_receipt_schema_rejected_before_native_metadata(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt = tmp_path / 'receipt.json'
    receipt.write_text(json.dumps(dict(schema='carla.moving_common10_conversion_receipt.v2', case=qualification.CASE, dataset_split='val')))
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256=qualification.sha(receipt),
        episode=[adapter.EpisodeInput('val', tmp_path / qualification.CASE / 'episode')],
        image_mode='copy', validate_mode='planning', output=tmp_path / qualification.PREPARED)
    with pytest.raises(adapter.AdapterError, match='exact VAL receipt'):
        qualification.authorize(args)


def test_actual_diagnostic_and_route_selection_pins(captured_dev):
    case, manifest = captured_dev
    audit_path = case.with_name(case.name + '_analysis') / 'summary.json'
    if not audit_path.is_file():
        pytest.skip('local immutable DEV raw audit is unavailable')
    assert qualification.sha(audit_path) == qualification.AUDIT_SHA
    assert len(qualification.companion_pins(audit_path)) == 3
    report = qualification.document(audit_path)
    assert report['schema'] == 'carla.moving_capture_diagnostic.v2'
    assert report['dataset_split_intent'] == 'val' and report['moving_scope_necessary_checks_clear'] is True
    assert report['route_selection_evidence'] == manifest['capture_contract']['moving_task']['route_selection_evidence']
    owner = qualification.document(case / 'owner_plan.json')
    ref = owner['moving_task_capture']['route_selection_evidence']
    assert qualification.sha(case / ref['archived_path']) == qualification.sha(ref['path']) == qualification.moving.SELECTION_SHA


@pytest.mark.parametrize('outcome', ['empty', 'unexpected_error', 'has_samples'])
def test_real_loader_empty_split_result_must_be_explicit(monkeypatch, outcome):
    calls = []

    def load(root, *, split, mode, check_image_hashes):
        calls.append(split)
        assert mode == 'planning' and check_image_hashes is True
        if outcome == 'empty':
            raise qualification.contract.ContractError(f"dataset split {split!r} contains no samples")
        if outcome == 'unexpected_error':
            raise qualification.contract.ContractError('corrupt image')
        return SimpleNamespace(examples=[object()])

    monkeypatch.setattr(qualification.dataset, 'load_training_examples', load)
    if outcome == 'empty':
        assert qualification.verify_empty_splits(Path('unused')) == dict(train='NO_SAMPLES', test='NO_SAMPLES')
        assert calls == ['train', 'test']
    else:
        with pytest.raises(adapter.AdapterError):
            qualification.verify_empty_splits(Path('unused'))
        assert calls == ['train']


def test_prepared_train_metadata_rejected_before_loader(tmp_path, monkeypatch):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    monkeypatch.setattr(qualification, 'recheck', lambda _grant: None)
    documents = {'dataset.json': dict(episodes=[dict(manifest='episodes/one/episode.json')]),
        'episode.json': dict(split='train', source_provenance=dict(adapter_id=adapter.ADAPTER_ID))}
    monkeypatch.setattr(qualification, 'document', lambda path: documents[path.name])
    monkeypatch.setattr(qualification.dataset, 'load_training_examples', lambda *_args, **_kwargs: pytest.fail('TRAIN dataset reached DEV loader'))
    with pytest.raises(adapter.AdapterError, match='wrong prepared split'):
        qualification.validate_prepared(tmp_path, None, {}, {})


def test_independent_target_math_is_unchanged():
    import inspect
    from scripts.e2e import qualify_carla_moving_common10 as legacy
    assert inspect.getsource(qualification.independently_expected) == inspect.getsource(legacy.independently_expected)


def test_source_closure_contains_both_adapters_and_helpers():
    assert {'scripts/e2e/qualify_carla_moving_dev_common10_v2.py',
        'scripts/e2e/prepare_carla_common10_dev_v2.py', 'scripts/e2e/prepare_carla_common10_dataset.py',
        'scripts/e2e/carla_moving_capture_dev_v2.py', 'scripts/e2e/carla_moving_capture.py'} <= set(qualification.SOURCE_PATHS)
