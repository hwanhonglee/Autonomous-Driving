"""HH_260906 - Check metadata/target-only readiness using existing Common10 fixtures, with unopened TEST targets."""
from __future__ import annotations

import ast
from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from portable_e2e import contract as c
from scripts.e2e import audit_portable_data_readiness as audit
# HH_260906 - Import fixtures from the canonical test namespace without the removed root alias.
from learning.tests.test_portable_e2e_contract import _write_dataset, _write_json, _sha, _mutate_first_sample

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


def rewrite_episode(root, index, change):
    manifest = json.loads((root / 'dataset.json').read_text())
    ref = manifest['episodes'][index]
    path = root / ref['manifest']
    episode = json.loads(path.read_text())
    change(episode, path.parent)
    _write_json(path, episode)
    ref['sha256'] = _sha(path)
    _write_json(root / 'dataset.json', manifest)


def independent_dataset(tmp_path, *, repeated_train=False):
    root = tmp_path / 'dataset'
    specs = (('train_a', 'train'), ('val_a', 'val'), ('test_a', 'test'))
    if repeated_train:
        specs += (('train_b', 'train'),)
    _write_dataset(root, episodes=specs)
    def map_change(index):
        def change(episode, folder):
            episode['map_id'] = episode['site_id'] = f'Town{index:02d}'
            semantic = c._route_geometry_semantic_hash(folder / episode['route_geometry_file'],
                episode=episode, contract=c.load_contract(), expected_sha256=episode['route_geometry_sha256'])
            episode['split_group_id'] = f"carla:{episode['map_id']}:{semantic}"
            episode['scene_group_id'] = f"{episode['split_group_id']}:traffic={episode['traffic_seed']}:weather={episode['weather']}"
        return change
    rewrite_episode(root, 1, map_change(3))
    rewrite_episode(root, 2, map_change(4))
    return root


def test_full_fixture_coverage_counts_anchors_and_independent_identities_separately(tmp_path):
    root = independent_dataset(tmp_path, repeated_train=True)
    report = audit.audit_dataset(root)
    train, val = report['splits']['train'], report['splits']['val']
    assert train['anchor_count'] == 22 and val['anchor_count'] == 11
    assert train['distinct_episode_id_count'] == 2
    assert train['distinct_split_group_id_count'] == train['distinct_site_id_count'] == train['distinct_weather_count'] == 1
    assert train['motion_groups']['continuing_motion']['anchor_count'] == 22
    assert len(report['route_groups']['train']) == 1
    assert report['test_metadata']['declared_sample_count'] == 11
    assert report['status'] == 'AUDITED_NOT_TRAINING_APPROVED'
    assert report['training_data_approved'] is report['automatic_promotion'] is report['vehicle_control_approved'] is False
    assert report['scope']['neural_inference'] is report['scope']['images_opened'] is False
    assert any(g.get('split') == 'val' and g.get('motion_group') == 'stationary_hold' for g in report['coverage_gaps'])
    assert all(report['source_sha256'][str(path.relative_to(audit.ROOT))] == audit.sha_file(path) for path in audit.SOURCE_PATHS)


def test_test_payloads_never_opened_even_when_missing(tmp_path, monkeypatch):
    root = independent_dataset(tmp_path)
    test_root = root / 'episodes/test_a'
    for name in ('samples.jsonl', 'source_manifest.json', 'collection_config.json', 'route_geometry.json'):
        (test_root / name).unlink()
    original_json, original_lines = c._read_json_and_sha256, c._iter_jsonl
    def guarded_json(path, *args, **kwargs):
        assert not Path(path).is_relative_to(test_root) or Path(path).name == 'episode.json'
        return original_json(path, *args, **kwargs)
    def guarded_lines(path, *args, **kwargs):
        assert not Path(path).is_relative_to(test_root)
        return original_lines(path, *args, **kwargs)
    monkeypatch.setattr(c, '_read_json_and_sha256', guarded_json)
    monkeypatch.setattr(c, '_iter_jsonl', guarded_lines)
    result = audit.audit_dataset(root)
    assert result['test_metadata']['target_behavior'] == 'NOT_OPENED'
    assert result['scope']['test_targets_opened'] is result['scope']['test_routes_opened'] is False
    assert not any('/test_a/' in name and not name.endswith('/episode.json') for name in result['input_sha256'])


def test_absent_ground_truth_is_unknown_and_false_is_not_an_empty_negative(tmp_path):
    root = independent_dataset(tmp_path)
    labels = audit.audit_dataset(root)['splits']['train']['task_label_availability']
    assert labels['objects'] == labels['occupancy'] == {'DECLARED_UNAVAILABLE': 11, 'NOT_RECORDED': 0}
    for task in ('lane', 'depth', 'traffic_light'):
        assert labels[task] == {'DECLARED_UNAVAILABLE': 0, 'NOT_RECORDED': 11}
    _mutate_first_sample(root, lambda row: row['labels'].update(lane={'available': False}))
    labels = audit.audit_dataset(root)['splits']['train']['task_label_availability']
    assert labels['lane'] == {'DECLARED_UNAVAILABLE': 1, 'NOT_RECORDED': 10}


@pytest.mark.parametrize('task', ['objects', 'occupancy', 'lane', 'depth', 'traffic_light', 'unknown_task'])
def test_task_true_or_unknown_schema_cannot_be_admitted(tmp_path, task):
    root = independent_dataset(tmp_path)
    _mutate_first_sample(root, lambda row: row['labels'].update({task: {'available': True}}))
    with pytest.raises(c.ContractError):
        audit.audit_dataset(root)


def test_warmup_and_navigation_commands_do_not_become_observed_turns(tmp_path):
    root = independent_dataset(tmp_path)
    def phase_config(episode, folder):
        config_path = folder / episode['source_provenance']['collection_config_file']
        config = json.loads(config_path.read_text())
        config['native_result'] = {'capture_phases': {
            'stationary_warmup': {'first_timestamp': 1., 'last_timestamp': 1.2},
            'driving': {'first_timestamp': 1.3, 'last_timestamp': 2.}}}
        _write_json(config_path, config)
        episode['source_provenance']['collection_config_sha256'] = _sha(config_path)
    rewrite_episode(root, 0, phase_config)
    _mutate_first_sample(root, lambda row: row['navigation'].update(command=0))
    report = audit.audit_dataset(root)
    train = report['splits']['train']
    assert train['recorded_capture_phase_counts']['stationary_warmup'] == 3
    assert train['recorded_capture_phase_counts']['driving'] == 8
    assert train['current_navigation_command_counts']['0'] == 1
    assert any(g.get('scenario') == 'left' and g.get('split') == 'train' for g in report['coverage_gaps'])
    assert report['episodes'][0]['declared_scenario_tags'] == ['lane_follow', 'straight']


@pytest.mark.parametrize('vx,speeds,valid,expected', [
    (0., [0.] * 64, [True] * 64, 'stationary_hold'),
    (.1, [.1] * 64, [True] * 64, 'stationary_hold'),
    (-.1, [.1] * 64, [True] * 64, 'stationary_hold'),
    (.1000000001, [.1] * 64, [True] * 64, 'moving_to_stop'),
    (2., [2.] * 54 + [.1] * 10, [True] * 64, 'moving_to_stop'),
    (2., [2.] * 55 + [0.] * 9, [True] * 64, 'other_motion'),
    (0., [.5] * 64, [True] * 64, 'continuing_motion'),
    (2., [.4999999999] * 64, [True] * 64, 'other_motion'),
    (0., [0.] + [None] * 63, [True] + [False] * 63, 'unavailable_masks'),
])
def test_rule_boundaries_match_frozen_classifier_without_importing_torch(vx, speeds, valid, expected):
    # HH_260906 - Extract only the two trusted pure functions, avoiding top-level Torch/runtime imports and model creation.
    path = _hh_source_path(audit.ROOT, 'scripts/e2e/audit_portable_stopmix_behavior.py')
    tree = ast.parse(path.read_text())
    selected = [node for node in tree.body if isinstance(node, ast.FunctionDef)
        and node.name in ('checked_targets', 'motion_group')]
    assert len(selected) == 2
    namespace = {'math': math, 'require': audit.require}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)
    assert audit.motion_group(vx, speeds, valid) == namespace['motion_group'](vx, speeds, valid) == expected


@pytest.mark.parametrize('vx,speeds,valid', [
    (True, [0.] * 64, [True] * 64), (math.nan, [0.] * 64, [True] * 64),
    (0., [math.inf] + [0.] * 63, [True] * 64), (0., [True] + [0.] * 63, [True] * 64),
    (0., [-1.] + [0.] * 63, [True] * 64), (0., [0.] * 64, [1] * 64),
    (0., [None] * 64, [False] * 64), (0., [0.] * 64, [False, True] + [False] * 62),
    (0., [0.] * 64, [True] + [False] * 63), (0., [0.] * 63, [True] * 63),
])
def test_invalid_numbers_and_masks_fail_closed(vx, speeds, valid):
    with pytest.raises(c.ContractError):
        audit.motion_group(vx, speeds, valid)


def test_masked_prefix_preserves_unknown_group_and_null_semantics(tmp_path):
    root = independent_dataset(tmp_path)
    def mask(sample):
        target = sample['labels']['planning']
        target['valid'] = [True] + [False] * 63
        for key in ('speed_mps', 'yaw_rad', 'target_timestamp_ns'):
            target[key][1:] = [None] * 63
        target['positions_base_xy_m'][1:] = [[None, None]] * 63
        target['invalid_reason'][1:] = ['episode_end'] * 63
    _mutate_first_sample(root, mask)
    report = audit.audit_dataset(root)
    assert report['splits']['train']['motion_groups']['unavailable_masks']['anchor_count'] == 1
    assert report['splits']['train']['valid_future_point_counts'] == {'1': 1, '64': 10}
    _mutate_first_sample(root, lambda row: row['labels']['planning']['positions_base_xy_m'].__setitem__(1, [0., 0.]))
    with pytest.raises(c.ContractError, match='masked nulls'):
        audit.audit_dataset(root)


def test_same_route_geometry_cannot_cross_train_val_even_with_different_file_bytes(tmp_path):
    root = tmp_path / 'dataset'
    _write_dataset(root, episodes=(('a', 'train'), ('b', 'val')))
    with pytest.raises(c.ContractError, match='split'):
        audit.audit_dataset(root)


def test_test_metadata_group_collision_is_detected_without_opening_test_targets(tmp_path):
    root = independent_dataset(tmp_path)
    train = json.loads((root / 'episodes/train_a/episode.json').read_text())
    def collision(episode, _):
        for key in ('map_id', 'split_group_id', 'scene_group_id'):
            episode[key] = train[key]
    rewrite_episode(root, 2, collision)
    with pytest.raises(c.ContractError, match='split'):
        audit.audit_dataset(root)


@pytest.mark.parametrize('case', ['dataset_denied', 'config_denied', 'metadata_hash', 'samples_hash', 'route_group'])
def test_admission_denial_and_tampering_fail_closed(tmp_path, case):
    root = independent_dataset(tmp_path)
    if case == 'dataset_denied':
        path = root / 'dataset.json'; value = json.loads(path.read_text()); value['training_data_approved'] = False
        _write_json(path, value)
    elif case == 'config_denied':
        def denied(episode, folder):
            path = folder / 'collection_config.json'
            _write_json(path, {'native_result': {'training_data_approved': False}})
            episode['source_provenance']['collection_config_sha256'] = _sha(path)
        rewrite_episode(root, 0, denied)
    elif case == 'metadata_hash':
        (root / 'episodes/train_a/collection_config.json').write_text('{}\n')
    elif case == 'samples_hash':
        with (root / 'episodes/train_a/samples.jsonl').open('a') as stream:
            stream.write('{}\n')
    else:
        def bad_group(episode, _):
            episode['split_group_id'] = 'carla:Town07:' + 'f' * 64
            episode['scene_group_id'] = episode['split_group_id'] + ':traffic=17:weather=ClearNoon'
        rewrite_episode(root, 0, bad_group)
    with pytest.raises(c.ContractError):
        audit.audit_dataset(root)


def test_postcheck_detects_source_change(tmp_path, monkeypatch):
    root = independent_dataset(tmp_path)
    original = audit.source_identity
    calls = 0
    def changed():
        nonlocal calls
        calls += 1
        result = original()
        if calls > 1:
            result['changed_source'] = 'f' * 64
        return result
    monkeypatch.setattr(audit, 'source_identity', changed)
    with pytest.raises(c.ContractError, match='source changed'):
        audit.audit_dataset(root)


@pytest.mark.parametrize('location', ['episode', 'config'])
def test_missing_or_null_metadata_hash_never_becomes_an_unpinned_read(tmp_path, location):
    root = independent_dataset(tmp_path)
    if location == 'episode':
        path = root / 'dataset.json'
        manifest = json.loads(path.read_text())
        manifest['episodes'][0].pop('sha256')
        _write_json(path, manifest)
    else:
        rewrite_episode(root, 0, lambda episode, _: episode['source_provenance'].update(collection_config_sha256=None))
    with pytest.raises(c.ContractError, match='requires a SHA256'):
        audit.audit_dataset(root)


@pytest.mark.parametrize('delta_ns', [1, -1_000_000_000])
def test_current_navigation_command_rejects_future_and_stale_timestamps(tmp_path, delta_ns):
    root = independent_dataset(tmp_path)
    _mutate_first_sample(root, lambda row: row['navigation'].update(command_timestamp_ns=row['anchor_timestamp_ns'] + delta_ns))
    with pytest.raises(c.ContractError, match='causal and fresh'):
        audit.audit_dataset(root)


def test_all_test_metadata_does_not_claim_train_val_targets_were_opened(tmp_path):
    root = tmp_path / 'dataset'
    _write_dataset(root, episodes=(('test_only', 'test'),))
    report = audit.audit_dataset(root)
    assert report['scope']['train_val_targets_opened'] is False
    assert report['splits']['train']['anchor_count'] == report['splits']['val']['anchor_count'] == 0


def test_same_source_sample_cannot_be_recounted_in_a_renamed_episode(tmp_path):
    root = independent_dataset(tmp_path, repeated_train=True)
    first = json.loads((root / 'episodes/train_a/samples.jsonl').read_text().splitlines()[0])
    def reused(episode, folder):
        sample_path = folder / episode['sample_jsonl']
        rows = [json.loads(line) for line in sample_path.read_text().splitlines()]
        previous = rows[0]['source_sample_id']
        rows[0]['source_sample_id'] = first['source_sample_id']
        sample_path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
        episode['sample_jsonl_sha256'] = _sha(sample_path)
        source_path = folder / episode['source_provenance']['source_manifest_file']
        source = json.loads(source_path.read_text())
        for row in source['selected_samples']:
            if row['source_sample_id'] == previous:
                row['source_sample_id'] = first['source_sample_id']
        _write_json(source_path, source)
        episode['source_provenance']['source_manifest_sha256'] = _sha(source_path)
    rewrite_episode(root, 3, reused)
    with pytest.raises(c.ContractError, match='source sample reused'):
        audit.audit_dataset(root)


def test_cli_creates_fresh_report_and_refuses_overwrite(tmp_path, capsys):
    root = independent_dataset(tmp_path)
    output = tmp_path / 'audit.json'
    audit.main(['--dataset', str(root), '--output', str(output)])
    assert 'AUDITED_NOT_TRAINING_APPROVED' in capsys.readouterr().out
    before = output.read_bytes()
    with pytest.raises(c.ContractError, match='fresh'):
        audit.main(['--dataset', str(root), '--output', str(output)])
    assert output.read_bytes() == before
    with pytest.raises(c.ContractError, match='outside'):
        audit.main(['--dataset', str(root), '--output', str(root / 'audit.json')])


def test_unsafe_input_paths_and_symlink_output_parent_refused(tmp_path):
    root = independent_dataset(tmp_path)
    rewrite_episode(root, 0, lambda episode, _: episode.update(sample_jsonl='../escape.jsonl'))
    with pytest.raises(c.ContractError):
        audit.audit_dataset(root)
    target = tmp_path / 'target'; target.mkdir()
    link = tmp_path / 'link'; link.symlink_to(target, target_is_directory=True)
    with pytest.raises(c.ContractError, match='symlink'):
        audit.main(['--dataset', str(root), '--output', str(link / 'audit.json')])
    assert not list(target.iterdir())


def test_current_scope_rejects_real_adapter_claims(tmp_path):
    root = independent_dataset(tmp_path)
    rewrite_episode(root, 0, lambda episode, _: episode.update(domain='real'))
    with pytest.raises(c.ContractError, match='current CARLA'):
        audit.audit_dataset(root)
