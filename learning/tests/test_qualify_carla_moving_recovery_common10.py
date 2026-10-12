"""HH_260906 - Fail-closed CPU coverage for sealed recovery Common10 qualification without conversion."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_moving_capture as moving
from scripts.e2e import prepare_carla_common10_dataset as adapter
from scripts.e2e import qualify_carla_moving_recovery_common10 as qualification

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


def sealed_plan(slot_id):
    """HH_260906 - Build one complete synthetic plan from an unchanged sealed slot contract."""
    spec = moving.slot_spec(slot_id)
    return moving.plan_contract(slot_id) | {
        'declared_at_utc': '2026-09-21T00:00:00+00:00',
        'output': spec['capture_path'],
        'input_paths': {'route_file': spec['route_path'], 'mapping': 'mapping', 'calibration': 'calibration'},
        'input_sha256': {'route_file': spec['route_sha256'], 'mapping': '1' * 64, 'calibration': '2' * 64},
        'source_sha256': {name: '3' * 64 for name in moving.SOURCE_PATHS},
    }


@pytest.mark.parametrize('value', [None, '', 'a' * 63, 'g' * 64, 'A' * 64, 123])
def test_external_hashes_are_mandatory_lowercase_sha256(value):
    """HH_260906 - Reject absent, malformed, uppercase, or non-string external pins."""
    with pytest.raises(adapter.AdapterError):
        qualification.hash_value(value)


@pytest.mark.parametrize('slot_id', qualification.QUALIFIABLE_SLOT_IDS)
def test_resolve_scope_accepts_each_and_only_sealed_recovery_slot(tmp_path, monkeypatch, slot_id):
    """HH_260906 - Resolve all four successful recovery slots through their immutable contract registry."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    monkeypatch.setattr(moving, 'verify_cohort_slot', lambda observed: observed == slot_id
                        or pytest.fail('wrong recovery slot verified'))
    plan = sealed_plan(slot_id)
    monkeypatch.setattr(qualification, 'document', lambda _path: plan)
    monkeypatch.setattr(qualification, 'sha', lambda _path: '4' * 64)
    case = tmp_path / Path(moving.slot_spec(slot_id)['capture_path']).parent
    scope = qualification.resolve_scope(case)
    assert scope['slot_id'] == slot_id
    assert scope['town'] == moving.slot_spec(slot_id)['town']
    assert scope['route_sha256'] == moving.slot_spec(slot_id)['route_sha256']


def test_resolve_scope_rejects_legacy_failed_and_mutated_recovery_contract(tmp_path, monkeypatch):
    """HH_260906 - Keep legacy T07-S01 and post-hoc fixed-field changes outside the new qualifier."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    monkeypatch.setattr(moving, 'verify_cohort_slot', lambda _slot: None)
    monkeypatch.setattr(qualification, 'sha', lambda _path: '4' * 64)
    legacy = moving.plan_contract('T07-S01') | {
        'declared_at_utc': '2026-09-21T00:00:00+00:00', 'output': moving.CAPTURE_PATH,
        'input_paths': {'route_file': moving.ROUTE_PATH, 'mapping': 'mapping', 'calibration': 'calibration'},
        'input_sha256': {'route_file': moving.ROUTE_SHA, 'mapping': '1' * 64, 'calibration': '2' * 64},
        'source_sha256': {name: '3' * 64 for name in moving.SOURCE_PATHS},
    }
    monkeypatch.setattr(qualification, 'document', lambda _path: legacy)
    with pytest.raises(adapter.AdapterError, match='legacy'):
        qualification.resolve_scope(tmp_path / Path(moving.CAPTURE_PATH).parent)
    failed = sealed_plan('T01-S02')
    monkeypatch.setattr(qualification, 'document', lambda _path: failed)
    with pytest.raises(adapter.AdapterError, match='failed'):
        qualification.resolve_scope(tmp_path / Path(moving.slot_spec('T01-S02')['capture_path']).parent)
    changed = sealed_plan('T07-S02'); changed['selected_progress_m'] = [31., 165.]
    monkeypatch.setattr(qualification, 'document', lambda _path: changed)
    with pytest.raises(adapter.AdapterError, match='contract changed'):
        qualification.resolve_scope(tmp_path / Path(moving.slot_spec('T07-S02')['capture_path']).parent)


def write_source_manifest_fixture(tmp_path):
    """HH_260906 - Create the minimum exact raw ledger required by source-manifest verification."""
    case = tmp_path / 'case'
    names = ('episode/manifest.json', 'episode/route.json', 'episode/states.jsonl',
             'episode/camera_frames.jsonl', 'episode/moving_task_plan.json',
             'owner_plan.json', 'owner_result.json')
    rows = []
    for index, name in enumerate(names):
        path = case / name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f'payload-{index}')
        rows.append({'path': name, 'sha256': qualification.adapter._sha256(path),
                     'size_bytes': path.stat().st_size})
    return case, {'source_manifest': rows}


def test_source_manifest_rehashes_every_critical_file_and_detects_mutation(tmp_path, monkeypatch):
    """HH_260906 - Refuse a rewritten case even when its audit ledger still claims the old digest."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    case, audit = write_source_manifest_fixture(tmp_path)
    seen = {}

    def pin(path, expected):
        digest = qualification.sha(path)
        if digest != expected:
            raise adapter.AdapterError('changed')
        seen[Path(path).relative_to(tmp_path).as_posix()] = digest

    qualification.pin_source_manifest(case, audit, pin)
    assert len(seen) == 7
    (case / 'episode/states.jsonl').write_text('rewritten')
    with pytest.raises(adapter.AdapterError, match='size changed|changed'):
        qualification.pin_source_manifest(case, audit, pin)


def test_source_manifest_rejects_traversal_and_duplicate_rows(tmp_path, monkeypatch):
    """HH_260906 - Reject a source ledger that aliases or escapes the authenticated case."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    case, audit = write_source_manifest_fixture(tmp_path)
    audit['source_manifest'].append(dict(audit['source_manifest'][0]))
    with pytest.raises(adapter.AdapterError, match='duplicate'):
        qualification.pin_source_manifest(case, audit, lambda *_args: None)
    case, audit = write_source_manifest_fixture(tmp_path)
    audit['source_manifest'][0]['path'] = '../foreign'
    with pytest.raises(adapter.AdapterError):
        qualification.pin_source_manifest(case, audit, lambda *_args: None)


def synthetic_target_rows(count=170):
    """HH_260906 - Build deterministic 20-Hz straight motion with enough measured future support."""
    rows = []
    for index in range(count):
        timestamp = index * .05
        rows.append({'frame': 1000 + index, 'timestamp': timestamp, 'x': timestamp, 'y': 0.,
                     'yaw': 0., 'vx': 1., 'vy': 0.})
    return rows


def test_independent_target_reconstruction_has_exact_64_points_and_timestamps():
    """HH_260906 - Reconstruct a full 6.4-second target without extrapolation or label input."""
    states = synthetic_target_rows()
    camera = {'frame': 1020, 'source_timestamps': {name: 1. for name in moving.CAMERAS}}
    selected = [{'frame': 1020, 'timestamp_ns': 1_000_000_000, 'measured_route_progress_m': 1.}]
    result = qualification.target_reconstruction(states, [camera], selected)
    assert len(result) == 1
    assert len(result[0]['positions_base_xy_m']) == len(result[0]['yaw_rad']) == len(result[0]['speed_mps']) == 64
    assert result[0]['target_timestamp_ns'] == [1_000_000_000 + (index + 1) * 100_000_000 for index in range(64)]
    assert result[0]['positions_base_xy_m'][0] == pytest.approx([.1, 0.])
    assert result[0]['positions_base_xy_m'][-1] == pytest.approx([6.4, 0.])
    assert result[0]['speed_mps'] == [1.] * 64


def test_independent_target_reconstruction_refuses_missing_actual_future():
    """HH_260906 - Never extrapolate the final part of a selected 64-point planning target."""
    states = synthetic_target_rows(100)
    camera = {'frame': 1020, 'source_timestamps': {name: 1. for name in moving.CAMERAS}}
    selected = [{'frame': 1020, 'timestamp_ns': 1_000_000_000, 'measured_route_progress_m': 1.}]
    with pytest.raises(adapter.AdapterError, match='actual bracketing native future'):
        qualification.target_reconstruction(states, [camera], selected)


def test_wrong_external_audit_pin_stops_before_case_scope_or_payload(tmp_path, monkeypatch):
    """HH_260906 - Authenticate external case evidence before reading the sealed plan or row payloads."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    case = tmp_path / 'case'; (case / 'episode').mkdir(parents=True)
    audit = tmp_path / 'case_analysis/summary.json'; audit.parent.mkdir(); audit.write_text('{}')
    (case / 'owner_plan.json').write_text('{}'); (case / 'episode/manifest.json').write_text('{}')
    monkeypatch.setattr(qualification, 'resolve_scope', lambda *_args: pytest.fail('scope read after bad audit pin'))
    companions = {name: 'a' * 64 for name in qualification.COMPANIONS}
    with pytest.raises(adapter.AdapterError, match='qualification input changed'):
        qualification.evidence(case, qualification.sha(case / 'episode/manifest.json'),
            qualification.sha(case / 'owner_plan.json'), audit, 'a' * 64, companions, decode_images=False)


def test_authorizer_rejects_bad_receipt_pin_before_document(tmp_path, monkeypatch):
    """HH_260906 - Refuse a changed receipt before trusting its slot, case, or output metadata."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt = tmp_path / 'receipt.json'; receipt.write_text('{}')
    monkeypatch.setattr(qualification, 'document', lambda *_args: pytest.fail('read bad receipt'))
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256='a' * 64)
    with pytest.raises(adapter.AdapterError, match='external SHA differs'):
        qualification.authorize(args)


def test_authorizer_rejects_v1_receipt_before_case_or_payload(tmp_path, monkeypatch):
    """HH_260906 - Leave existing v1 receipts intact but make their superseded schema fail closed."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt_doc = {'schema': 'carla.moving_recovery_common10_conversion_receipt.v1', 'slot_id': 'T07-S02'}
    receipt = tmp_path / 'receipt_v1.json'; receipt.write_text(json.dumps(receipt_doc))
    monkeypatch.setattr(qualification, 'evidence', lambda *_args, **_kwargs: pytest.fail('read case for stale receipt'))
    args = SimpleNamespace(moving_qualification=receipt,
                           moving_qualification_sha256=qualification.sha(receipt))
    with pytest.raises(adapter.AdapterError, match='stale or foreign'):
        qualification.authorize(args)


def test_authorizer_rejects_non_train_before_reconstructing_evidence(tmp_path, monkeypatch):
    """HH_260906 - Keep validation and TEST episodes outside the recovery conversion path."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    receipt_doc = {'schema': qualification.SCHEMA, 'slot_id': 'T07-S02'}
    receipt = tmp_path / 'receipt.json'; receipt.write_text(json.dumps(receipt_doc))
    monkeypatch.setattr(qualification, 'document', lambda _path: receipt_doc)
    monkeypatch.setattr(qualification, 'evidence', lambda *_args, **_kwargs: pytest.fail('reconstructed non-TRAIN evidence'))
    episode = tmp_path / Path(moving.slot_spec('T07-S02')['capture_path'])
    args = SimpleNamespace(moving_qualification=receipt, moving_qualification_sha256=qualification.sha(receipt),
                           episode=[adapter.EpisodeInput('val', episode)], image_mode='copy',
                           validate_mode='planning', output=tmp_path / 'unused')
    with pytest.raises(adapter.AdapterError, match='TRAIN input'):
        qualification.authorize(args)


def test_prepared_paths_are_slot_specific_and_api_surface_is_complete():
    """HH_260906 - Preserve adapter-compatible hooks while preventing cross-slot output reuse."""
    paths = {qualification.prepared_path(slot_id) for slot_id in qualification.QUALIFIABLE_SLOT_IDS}
    assert len(paths) == len(qualification.QUALIFIABLE_SLOT_IDS)
    for name in ('authorize', 'verify_native_scope', 'verify_weather_provenance',
                 'native_route_id', 'validate_prepared', 'evidence'):
        assert callable(getattr(qualification, name))


@pytest.mark.parametrize('qualified,has_hook,route_weather,capture_weather,allowed', [
    (False, True, 'ClearNoon', 'CloudyNoon', False),
    (True, False, 'ClearNoon', 'CloudyNoon', False),
    (True, True, 'ClearNoon', 'CloudyNoon', True),
    (True, True, 'ClearNoon', 'ClearNoon', True),
])
def test_base_weather_hook_preserves_unqualified_and_legacy_rejection(
        qualified, has_hook, route_weather, capture_weather, allowed):
    """HH_260906 - Delegate weather provenance only to an explicit recovery authority, including clear captures."""
    calls = []

    def verify(*args):
        """HH_260906 - Record the exact receipt-bound weather hook invocation."""
        calls.append(args)

    authority = SimpleNamespace(**({'verify_weather_provenance': verify} if has_hook else {}))
    spec = SimpleNamespace(path=Path('/tmp/native'))
    manifest = {'runtime': {'town': 'Town07', 'weather': capture_weather}}
    route = {'town': 'Town07', 'weather': route_weather}
    grant = {'receipt': True} if qualified else None
    if allowed:
        qualification.adapter._verify_native_weather_provenance(spec, manifest, route, grant, authority)
        assert len(calls) == 1
    else:
        with pytest.raises(adapter.AdapterError, match='weather disagree'):
            qualification.adapter._verify_native_weather_provenance(spec, manifest, route, grant, authority)
        assert not calls


def test_base_weather_hook_never_overrides_town_mismatch():
    """HH_260906 - Reject map identity mismatch before any recovery weather authority is invoked."""
    authority = SimpleNamespace(verify_weather_provenance=lambda *_args: pytest.fail('hook called'))
    spec = SimpleNamespace(path=Path('/tmp/native'))
    manifest = {'runtime': {'town': 'Town01', 'weather': 'CloudyNoon'}}
    route = {'town': 'Town07', 'weather': 'ClearNoon'}
    with pytest.raises(adapter.AdapterError, match='town disagree'):
        qualification.adapter._verify_native_weather_provenance(spec, manifest, route, {'receipt': True}, authority)


@pytest.mark.parametrize('catalog_weather,capture_weather,mismatch', [
    ('ClearNoon', 'CloudyNoon', True), ('ClearNoon', 'ClearNoon', False),
])
def test_route_identity_records_immutable_catalog_and_actual_capture_weather(
        tmp_path, monkeypatch, catalog_weather, capture_weather, mismatch):
    """HH_260906 - Record catalog/capture weather truth without changing either authenticated source."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    case = tmp_path / 'case'; (case / 'episode').mkdir(parents=True)
    route = {'town': 'Town07', 'scenario': 'straight', 'coordinate_reference': 'base_link',
             'weather': catalog_weather}
    route_path = case / 'episode/route.json'; route_path.write_text(json.dumps(route))
    route_sha = qualification.sha(route_path)
    catalog = tmp_path / 'catalog/route.json'; catalog.parent.mkdir(); catalog.write_text(json.dumps(route))
    owner = {'route_path': str(catalog), 'route_sha256': route_sha,
             'moving_task_capture': {'plan': {'sha256': 'p' * 64},
                                     'inputs': {'route_file': {'path': str(catalog), 'sha256': route_sha}}}}
    owner_path = case / 'owner_plan.json'; owner_path.write_text(json.dumps(owner))
    scope = {'town': 'Town07', 'route_path': 'catalog/route.json', 'route_sha256': route_sha,
             'route_id': 'route', 'plan_sha256': 'p' * 64,
             'plan': {'fixed_arguments': {'weather': capture_weather}}}
    manifest = {'runtime': {'weather': capture_weather},
                'provenance': {'source_route': str(catalog), 'route_sha256': route_sha}}
    identity = qualification.direct_route_identity(case, manifest, route, owner, scope,
                                                   qualification.sha(owner_path))
    assert identity['catalog_route_weather'] == catalog_weather
    assert identity['capture_weather'] == capture_weather
    assert identity['catalog_capture_weather_mismatch'] is mismatch
    assert 'No route or manifest bytes were normalized.' in identity['weather_provenance_notice']


def test_route_identity_rejects_runtime_weather_outside_sealed_owner_plan(tmp_path, monkeypatch):
    """HH_260906 - Refuse capture weather that is not authenticated by the prospective owner plan."""
    monkeypatch.setattr(qualification, 'ROOT', tmp_path)
    case = tmp_path / 'case'; (case / 'episode').mkdir(parents=True)
    route = {'town': 'Town07', 'scenario': 'straight', 'coordinate_reference': 'base_link', 'weather': 'ClearNoon'}
    route_path = case / 'episode/route.json'; route_path.write_text(json.dumps(route)); route_sha = qualification.sha(route_path)
    catalog = tmp_path / 'catalog/route.json'; catalog.parent.mkdir(); catalog.write_text(json.dumps(route))
    owner = {'route_path': str(catalog), 'route_sha256': route_sha,
             'moving_task_capture': {'plan': {'sha256': 'p' * 64},
                                     'inputs': {'route_file': {'path': str(catalog), 'sha256': route_sha}}}}
    owner_path = case / 'owner_plan.json'; owner_path.write_text(json.dumps(owner))
    scope = {'town': 'Town07', 'route_path': 'catalog/route.json', 'route_sha256': route_sha,
             'route_id': 'route', 'plan_sha256': 'p' * 64,
             'plan': {'fixed_arguments': {'weather': 'CloudyNoon'}}}
    manifest = {'runtime': {'weather': 'WetNoon'},
                'provenance': {'source_route': str(catalog), 'route_sha256': route_sha}}
    with pytest.raises(adapter.AdapterError, match='capture weather differs'):
        qualification.direct_route_identity(case, manifest, route, owner, scope,
                                            qualification.sha(owner_path))


@pytest.mark.parametrize('case_name,manifest_sha,owner_sha,audit_sha,jpeg_sha,future_sha,projection_sha,'
                         'capture_weather,mismatch,raw_anchors,raw_images', [
    ('02_moving_t07_s02', '2bb19cf3d4ff81bd712a94a8a50ee4001282310e556f36f46041571ec42a1217',
     'f260bf0227d460e4f66f5e04760097e0d3245fce81ef72a0d5c7cde0e160867e',
     'fc80462b45172e9f9f965dc80ce52fe378af3e04ce274101fa9d476ce1b5be71',
     'c8c1088d7c88ea4902f0a80312e8586ba1c994f9d255863ba9349fda0f231cf6',
     'c35d3ef1afd2f8875d39e8be7940960d4b4536a4ba5b743736fea211ec5bb107',
     '4da326083a762a7c7379641662372a71b4f84284e1a73e488f2d26cafc1297d3', 'CloudyNoon', True, 578, 3468),
    ('03_moving_ct_s01', '7b4f28ff20b4d25434044403fc674a19bde419febc7a6cb46ebd23339385d1e5',
     '5e4dc1c4ad6a6441c905f2c7f618d71b4ba5599b41dad3aa56413ba09af0f750',
     '7ae493912c15e0538fd921d877bab31ad0da213b8ea9c3627c3afb6564329762',
     '0bec752f6a26607f927862f0577be0b295b2bcb59c74ab269414a00a28a74cdf',
     '17f3c0225e0a707027963698b516532b567fd311e56416cb42ad3dad3d4594ba',
     '3aa1fdf797d2de6038f5ec7d098709fca819a5a218ef8d632a39a9130b13fe42', 'ClearNoon', False, 576, 3456),
    ('04_moving_ct_s02', '88b04b8446aeb92a997323cbd5dd3eb2e7cbfb5de947c80a03f16c0af0fd9eb5',
     '9e3d7dc580d850d2649467d4bf8b91366721362dfc4c87bfd816e18da18f7752',
     'c2d95db48381997c0f7c5cf74d350de8aff8ecf503d821cbe838fe70589909eb',
     '3e1ffd5039e916658382fe469ed95eb4fb3bb2628305665389a483ed05ec5594',
     '032611630a1574201985458cfcab71cdfef086e26d4488c02121f62cab36c136',
     '83db91b9c520897113077d328d74ad99454fabf7e7f2ad663d2ffd53ed661079', 'CloudyNoon', True, 576, 3456),
    ('05_moving_t01_s01', 'c3f5f25e3f9c9c66dc50b031f22cfee3b30e40ce3637bb6fff6dbed23fd10678',
     '470c19d4b3f5ecb283c449ef8de29bcf1d0bcefb4e155cd68da933e405a07e9d',
     '759ff1318e4cf833a8222cc2f03aba8b233948313384d90f54f8e7fbca95589f',
     '3cb787b43a0afb448e535bbcdbd6a33ae6f78dee01e9ebca68d7a72fd918a012',
     '233dde700fc6c878e32ac334e48f2470e9087121d66d36aa64e912b128a4ad3e',
     '69b9f002f3b906774e78c3d1ad7c9f6cacd3518172f5ffc02df32900031dbdf1', 'ClearNoon', False, 576, 3456),
])
def test_current_successful_evidence_reconstructs_v2_weather_read_only_when_present(
        case_name, manifest_sha, owner_sha, audit_sha, jpeg_sha, future_sha, projection_sha,
        capture_weather, mismatch, raw_anchors, raw_images):
    """HH_260906 - Recheck all successful captures in one v2 schema without writing receipts or converting data."""
    base = _hh_source_path(qualification.ROOT, qualification.BASE)
    case = base / case_name; audit = base / (case_name + '_analysis') / 'summary.json'
    if not case.is_dir() or not audit.is_file():
        pytest.skip('current sealed recovery evidence is not present')
    result = qualification.evidence(case, manifest_sha, owner_sha, audit, audit_sha,
        {'jpeg_audit.jsonl': jpeg_sha, 'future_anchor_audit.jsonl': future_sha,
         'native_projection.jsonl': projection_sha}, decode_images=False)
    assert result['status'] == 'QUALIFIED_FOR_COMMON10_CONVERSION'
    assert result['selected_camera_anchors'] == 328
    assert result['target_points'] == 20_992
    assert result['raw_camera_anchors'] == raw_anchors
    assert result['raw_images'] == raw_images
    assert result['schema'] == qualification.SCHEMA
    assert result['route_identity']['catalog_route_weather'] == 'ClearNoon'
    assert result['route_identity']['capture_weather'] == capture_weather
    assert result['route_identity']['catalog_capture_weather_mismatch'] is mismatch
    qualification.verify_weather_provenance(
        adapter.EpisodeInput('train', case / 'episode'),
        qualification.document(case / 'episode/manifest.json'), qualification.document(case / 'episode/route.json'),
        {'document': result})
