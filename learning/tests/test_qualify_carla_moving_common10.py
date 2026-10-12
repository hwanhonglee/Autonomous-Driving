"""HH_260906 - CPU-only synthetic coverage for exact moving-task conversion; no actual capture is converted here."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_carla_common10_dataset as adapter
from scripts.e2e import qualify_carla_moving_common10 as qualification


def legacy_fixture():
    path = Path(__file__).with_name('test_prepare_carla_common10_dataset.py')
    spec = importlib.util.spec_from_file_location('moving_legacy_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    monkeypatch.setattr(adapter, '_jsonl', lambda *_: pytest.fail('denied native rows opened'))
    with pytest.raises(adapter.AdapterError, match='prohibits'):
        adapter._load_native(adapter.EpisodeInput('train', path), moving_qualification={})


@pytest.mark.parametrize('split', ['val', 'test'])
def test_authorizer_rejects_non_train_before_native_payload(tmp_path, monkeypatch, split):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256=qualification.sha(receipt),
        episode=[adapter.EpisodeInput(split, tmp_path / 'native')])
    monkeypatch.setattr(qualification, 'document', lambda *_: pytest.fail('non-TRAIN metadata opened'))
    with pytest.raises(adapter.AdapterError, match='TRAIN input'):
        qualification.authorize(args)


@pytest.mark.parametrize('mode', ['hardlink', 'runtime'])
def test_authorizer_requires_copy_planning(tmp_path, monkeypatch, mode):
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    monkeypatch.setattr(qualification, 'CASE', 'case')
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256=qualification.sha(receipt),
        episode=[adapter.EpisodeInput('train', tmp_path / 'case/episode')],
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


def make_synthetic_task(tmp_path, monkeypatch):
    fixture = legacy_fixture()
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    monkeypatch.setattr(qualification, 'CASE', 'case')
    monkeypatch.setattr(qualification, 'PREPARED', 'prepared')
    episode = fixture.write_native_episode(tmp_path / 'case/episode', duration_s=57.4)
    states = fixture._read_jsonl(episode / 'states.jsonl')
    cameras = fixture._read_jsonl(episode / 'camera_frames.jsonl')
    manifest = adapter._json(episode / 'manifest.json')
    route = adapter._json(episode / 'route.json')
    for i, row in enumerate(states):
        phase = 'stationary_warmup' if i < 70 else 'driving' if i < 1019 else 'stationary_tail'
        row.update(capture_phase=phase, x=max(0., min(i - 70, 948)) * .1,
                   vx=2. if phase == 'driving' else 0.)
        row['route_progress_m'] = row['x']
    for camera in cameras:
        camera['capture_phase'] = states[camera['frame'] - 10000]['capture_phase']
    route['route_length_m'] = 110.; route['goal_ros_pose']['x'] = 110.
    route['route'] = [dict(x=float(i), y=0., distance_m=float(i), vad_command=3) for i in range(111)]
    # HH_260906 - Match the actual direct catalog shape: no synthesized coordinate_alignment metadata.
    route.pop('coordinate_alignment')
    result = manifest['result']; result.update(goal_reached=False, task_complete=True, route_length_m=110.,
        final_route_progress_m=94.80000000000001, post_capture_route_progress_m=94.80000000000001,
        final_remaining_route_m=110. - 94.80000000000001, final_catalog_goal_distance_m=110. - 94.80000000000001,
        **qualification.moving.pending_scope())
    result['moving_task'] = qualification.moving.pending_scope()
    manifest['capture_contract']['agent_initialization'] = qualification.moving.pending_scope()
    manifest['capture_contract']['moving_task'] = {'plan_sha256': qualification.PLAN_SHA}
    for phase in adapter.CAPTURE_PHASES:
        ns = sum(r['capture_phase'] == phase for r in states); nc = sum(r['capture_phase'] == phase for r in cameras)
        result['capture_phases'][phase] = dict(state_count=ns, camera_anchor_count=nc)
        key = 'maximum_ticks' if phase == 'driving' else 'scheduled_ticks'
        manifest['capture_contract']['capture_phase_schedule'][phase][key] = ns
    fixture._write_json(episode / 'route.json', route)
    fixture._write_jsonl(episode / 'states.jsonl', states); fixture._write_jsonl(episode / 'camera_frames.jsonl', cameras)
    manifest['provenance']['route_sha256'] = qualification.sha(episode / 'route.json')
    manifest['provenance']['source_route'] = str(tmp_path / qualification.moving.ROUTE_PATH)
    fixture._write_json(episode / 'manifest.json', manifest)
    owner = dict(route_path=manifest['provenance']['source_route'], route_sha256=manifest['provenance']['route_sha256'],
        moving_task_capture={'inputs': {'route_file': {'path': manifest['provenance']['source_route'],
            'sha256': manifest['provenance']['route_sha256']}}})
    fixture._write_json(episode.parent / 'owner_plan.json', owner)
    monkeypatch.setattr(qualification, 'OWNER_SHA', qualification.sha(episode.parent / 'owner_plan.json'))
    monkeypatch.setattr(qualification.moving, 'ROUTE_SHA', manifest['provenance']['route_sha256'])
    selected = cameras[118:446]
    doc = dict(native_manifest_sha256=qualification.sha(episode / 'manifest.json'), plan_sha256=qualification.PLAN_SHA,
        conversion_only=True, route_identity=qualification.direct_route_identity(episode.parent, manifest, route, owner),
        selected_camera_frames=[r['frame'] for r in selected],
        selected_anchors=[dict(frame=r['frame'], timestamp_ns=round(r['timestamp'] * 1e9)) for r in selected],
        source_sha256={}, input_sha256={p.relative_to(tmp_path).as_posix(): qualification.sha(p)
            for p in (episode / 'images').rglob('*.jpg')}, full_raw_failures_retained={'fixture_original_failures': 7})
    receipt = tmp_path / 'receipt.json'; fixture._write_json(receipt, doc)
    grant = dict(receipt_path=str(receipt), receipt_sha256=qualification.sha(receipt), document=doc)
    args = fixture._args(adapter, episode, tmp_path / 'prepared')
    args.image_mode = 'copy'; args.moving_qualification = receipt; args.moving_qualification_sha256 = grant['receipt_sha256']
    # HH_260906 - Authentication is separately unit-tested; this synthetic corpus exercises real converter/validator/loader/math.
    monkeypatch.setattr(qualification, 'authorize', lambda _args: grant)
    return args, grant, states, manifest


def test_full_synthetic_moving_conversion_unchanged_validator_loader_and_all_targets(tmp_path, monkeypatch):
    args, grant, _states, manifest = make_synthetic_task(tmp_path, monkeypatch)
    original = (args.episode[0].path / 'manifest.json').read_bytes()
    output, report = adapter.prepare_dataset(args)
    final = adapter._json(output / 'moving_conversion_qualification.json')
    assert report['sample_count'] == final['train_examples'] == 328
    assert final['qualified_for_common10_planning_conversion'] is True
    assert final['rederived_target_points'] == 20992
    assert final['raw_image_hashes_preserved'] == 3450
    assert final['context_only_camera_anchors'] == 247
    assert final['independently_copied_selected_images'] == 1968
    assert final['full_raw_failures_retained'] == {'fixture_original_failures': 7}
    assert not final['native_goal_reached'] and not final['model_or_vehicle_release_approved']
    ep = next((output / 'episodes').iterdir())
    assert (ep / 'native_manifest.json').read_bytes() == original
    assert (ep / 'native_route.json').read_bytes() == (args.episode[0].path / 'route.json').read_bytes()
    assert final['route_identity']['coordinate_alignment_synthesized'] is False
    assert final['native_route_bytes_unchanged'] is True
    assert (args.episode[0].path / 'manifest.json').read_bytes() == original
    assert adapter._json(ep / 'collection_config.json')['native_result'] == manifest['result']
    assert adapter._json(ep / 'episode.json')['source_provenance']['adapter_id'] != adapter.ADAPTER_ID
    with pytest.raises(adapter.AdapterError, match='must not already exist'):
        adapter.prepare_dataset(args)


@pytest.mark.parametrize('field', ['final_remaining_route_m', 'final_catalog_goal_distance_m'])
def test_moving_goal_exception_keeps_factual_distance_assertions(tmp_path, monkeypatch, field):
    args, grant, _states, manifest = make_synthetic_task(tmp_path, monkeypatch)
    manifest['result'][field] += 1.
    path = args.episode[0].path / 'manifest.json'; path.write_text(json.dumps(manifest))
    grant['document']['native_manifest_sha256'] = qualification.sha(path)
    native = adapter._load_native(args.episode[0], moving_qualification=grant)
    with pytest.raises(adapter.AdapterError, match=field):
        adapter._state_timeline(native)


def test_same_goal_false_source_still_rejected_without_optin(tmp_path, monkeypatch):
    args, _grant, _states, _manifest = make_synthetic_task(tmp_path, monkeypatch)
    with pytest.raises(adapter.AdapterError, match='did not reach'):
        adapter._load_native(args.episode[0])


def test_prepared_target_corruption_fails_and_staged_output_is_removed(tmp_path, monkeypatch):
    args, _grant, _states, _manifest = make_synthetic_task(tmp_path, monkeypatch)
    original = qualification.independently_expected
    def corrupt(native, sample, *remaining):
        sample['labels']['planning']['positions_base_xy_m'][3][0] += .01
        return original(native, sample, *remaining)
    monkeypatch.setattr(qualification, 'independently_expected', corrupt)
    with pytest.raises(adapter.AdapterError, match='reconstruction differs'):
        adapter.prepare_dataset(args)
    assert not args.output.exists() and not args.output.with_name(args.output.name + '.partial').exists()


def test_missing_selected_anchor_is_not_silently_filtered(tmp_path, monkeypatch):
    args, grant, _states, _manifest = make_synthetic_task(tmp_path, monkeypatch)
    grant['document']['selected_camera_frames'] = grant['document']['selected_camera_frames'][:-1]
    with pytest.raises(adapter.AdapterError, match='328'):
        adapter.prepare_dataset(args)
    assert not args.output.exists()


def test_direct_catalog_shape_requires_qualified_identity_not_legacy_alignment(tmp_path, monkeypatch):
    args, grant, _states, manifest = make_synthetic_task(tmp_path, monkeypatch)
    route = adapter._json(args.episode[0].path / 'route.json')
    assert 'coordinate_alignment' not in route
    with pytest.raises(adapter.AdapterError, match='coordinate_alignment'):
        adapter._native_route_id(route)
    assert qualification.native_route_id(args.episode[0], manifest, route, grant) == 'town07_straight_s0000_p00'


@pytest.mark.parametrize('mutation', ['owner_path', 'source_path', 'source_hash', 'route_hash', 'alignment', 'receipt_identity'])
def test_qualified_direct_identity_rejects_inconsistent_sources(tmp_path, monkeypatch, mutation):
    args, grant, _states, manifest = make_synthetic_task(tmp_path, monkeypatch)
    spec = args.episode[0]; route = adapter._json(spec.path / 'route.json')
    if mutation == 'owner_path':
        path = spec.path.parent / 'owner_plan.json'; owner = adapter._json(path)
        owner['route_path'] = str(tmp_path / 'foreign_route.json'); path.write_text(json.dumps(owner))
        monkeypatch.setattr(qualification, 'OWNER_SHA', qualification.sha(path))
    elif mutation == 'source_path':
        manifest['provenance']['source_route'] = str(tmp_path / 'foreign_route.json')
    elif mutation == 'source_hash':
        manifest['provenance']['route_sha256'] = 'a' * 64
    elif mutation == 'route_hash':
        path = spec.path / 'route.json'; path.write_bytes(path.read_bytes() + b' ')
    elif mutation == 'alignment':
        route['coordinate_alignment'] = {'fabricated': True}
    else:
        grant['document']['route_identity']['route_id'] = 'town07_straight_s9999_p00'
    with pytest.raises(adapter.AdapterError, match='route'):
        qualification.native_route_id(spec, manifest, route, grant)
