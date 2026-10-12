#!/usr/bin/env python3
"""HH_260906 - Audit existing Common10 TRAIN/VAL coverage without opening TEST targets or admitting data."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from portable_e2e import contract as c
from portable_e2e import dataset as data

SCHEMA = 'portable_e2e.data_readiness_audit.v1'
GROUPS = ('unavailable_masks', 'stationary_hold', 'moving_to_stop', 'continuing_motion', 'other_motion')
PHASES = ('stationary_warmup', 'driving', 'stationary_tail', 'NOT_RECORDED')
TASKS = ('objects', 'occupancy', 'lane', 'depth', 'traffic_light')
# HH_260906 - Canonicalize owned module locations, never user-supplied evidence or dataset paths.
SOURCE_PATHS = (Path(__file__).resolve(), Path(c.__file__).resolve(), Path(data.__file__).resolve(),
    ROOT / 'tools/e2e/audit_portable_stopmix_behavior.py', c.DEFAULT_CONTRACT_PATH.resolve())


def require(value, message):
    if not value:
        raise c.ContractError(message)


def nonsymlink(path):
    path = Path(path).absolute()
    require(not any(p.is_symlink() for p in (path, *path.parents)), 'symlink paths are not supported')
    return path


def sha_file(path):
    path = nonsymlink(path)
    descriptor, before = c._open_regular_read_only(path, 'readiness input')
    digest = hashlib.sha256()
    try:
        while block := os.read(descriptor, c.READ_CHUNK_BYTES):
            digest.update(block)
        c._verify_open_file_identity(descriptor, path, before, 'readiness input')
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def source_identity():
    return {str(path.relative_to(ROOT)): sha_file(path) for path in SOURCE_PATHS}


def motion_group(current_vx, target_speed, valid):
    # HH_260906 - Pure copy of the frozen behavior rules; parity tests avoid importing its Torch/model stack.
    require(type(current_vx) in (int, float) and math.isfinite(current_vx), 'current raw vx must be finite')
    require(isinstance(target_speed, list) and isinstance(valid, list) and len(target_speed) == len(valid) == 64
        and all(type(v) is bool for v in valid), 'target mask must contain 64 booleans')
    require(any(valid) and valid == sorted(valid, reverse=True), 'target mask must be a nonempty prefix')
    require(all(type(v) in (int, float) and math.isfinite(v) and v >= 0
        for v, ok in zip(target_speed, valid) if ok), 'valid target speeds must be finite nonnegative numbers')
    require(all(v is None for v, ok in zip(target_speed, valid) if not ok), 'masked speed targets must remain null')
    if not all(valid):
        return 'unavailable_masks'
    if abs(current_vx) <= .1 and all(v <= .1 for v in target_speed):
        return 'stationary_hold'
    if current_vx > .1 and all(v <= .1 for v in target_speed[-10:]):
        return 'moving_to_stop'
    if all(v >= .5 for v in target_speed):
        return 'continuing_motion'
    return 'other_motion'


def planning_group(sample, episode):
    labels = c._mapping(sample.get('labels'), 'labels')
    require(not (set(labels) - {'planning', *TASKS}), 'unsupported task labels in current Common10 scope')
    planning = c._mapping(labels.get('planning'), 'planning')
    require(planning.get('available') is True, 'planning must be available')
    require(c._number(planning.get('dt_s'), 'planning.dt_s') == .1, 'planning grid must remain 100 ms')
    ego = c._mapping(sample.get('ego'), 'ego')
    vx = c._vector(ego.get('linear_velocity_base_mps'), 3, 'ego velocity')[0]
    group = motion_group(vx, planning.get('speed_mps'), planning.get('valid'))
    for key in ('positions_base_xy_m', 'yaw_rad', 'target_timestamp_ns', 'invalid_reason'):
        require(isinstance(planning.get(key), list) and len(planning[key]) == 64, 'all planning arrays must contain64 points')
    anchor = c._integer(sample.get('anchor_timestamp_ns'), 'anchor', 0)
    for index, valid in enumerate(planning['valid']):
        xy, yaw, stamp, reason = (planning[k][index] for k in
            ('positions_base_xy_m', 'yaw_rad', 'target_timestamp_ns', 'invalid_reason'))
        if valid:
            c._vector(xy, 2, 'target XY'); c._number(yaw, 'target yaw')
            require(c._integer(stamp, 'target timestamp', 0) == anchor + (index + 1) * 100_000_000
                and stamp <= episode['end_timestamp_ns'] and reason is None, 'valid future timing differs')
        else:
            require(xy == [None, None] and yaw is stamp is None
                and reason in ('episode_end', 'sensor_gap', 'label_unavailable'), 'invalid targets must remain masked nulls')
    availability = {}
    for task in TASKS:
        if task not in labels:
            require(task not in ('objects', 'occupancy'), 'current Common10 task availability must be explicit')
            availability[task] = 'NOT_RECORDED'
        else:
            value = c._mapping(labels[task], 'labels.' + task)
            require(set(value) == {'available'} and value['available'] is False,
                'nonplanning task payloads cannot be approved by this readiness audit')
            availability[task] = 'DECLARED_UNAVAILABLE'
    return group, sum(planning['valid']), availability


def phase_windows(config):
    native = config.get('native_result')
    if native is None:
        return []
    native = c._mapping(native, 'native_result')
    recorded = c._mapping(native.get('capture_phases', {}), 'capture_phases')
    windows = []
    for phase in PHASES[:-1]:
        item = recorded.get(phase)
        if item is None:
            continue
        require(isinstance(item, dict), 'capture phase metadata must be an object')
        first, last = item.get('first_timestamp'), item.get('last_timestamp')
        if first is last is None:
            continue
        first = round(c._number(first, 'phase first timestamp', 0.) * 1e9)
        last = round(c._number(last, 'phase last timestamp', 0.) * 1e9)
        require(first <= last and (not windows or first > windows[-1][2]), 'capture phase windows overlap or reverse')
        windows.append((phase, first, last))
    return windows


def summarize_rows(rows):
    # HH_260906 - Counts are overlapping anchors and distinct recorded identities, never independent stopping events.
    def aggregate(selected):
        result = {'anchor_count': len(selected)}
        for field in ('episode_id', 'split_group_id', 'site_id', 'weather'):
            values = sorted({row[field] for row in selected})
            result[field + 's'] = values
            result['distinct_' + field + '_count'] = len(values)
        result['recorded_capture_phase_counts'] = {p: sum(r['capture_phase'] == p for r in selected) for p in PHASES}
        result['current_navigation_command_counts'] = {str(i): sum(r['command'] == i for r in selected) for i in range(6)}
        return result
    result = aggregate(rows)
    result.update(motion_groups={group: aggregate([r for r in rows if r['motion_group'] == group]) for group in GROUPS},
        valid_future_point_counts=dict(sorted(Counter(str(r['valid_points']) for r in rows).items())),
        task_label_availability={task: {state: sum(r['labels'][task] == state for r in rows)
            for state in ('DECLARED_UNAVAILABLE', 'NOT_RECORDED')} for task in TASKS})
    return result


def audit_dataset(dataset):
    root = nonsymlink(dataset).resolve(strict=True)
    require(root.is_dir(), 'dataset directory required')
    sources = source_identity()
    inputs = {}
    def read(path, expected=None):
        path = nonsymlink(path)
        require(path.is_relative_to(root), 'metadata escapes dataset')
        require(expected is not None or path == root / 'dataset.json', 'metadata reference requires a SHA256')
        value, actual = c._read_json_and_sha256(path)
        require(expected is None or actual == c._sha256(expected, 'expected SHA'), 'metadata hash mismatch')
        name = path.relative_to(root).as_posix()
        require(name not in inputs or inputs[name] == actual, 'metadata changed between reads')
        inputs[name] = actual
        return value
    manifest = read(root / 'dataset.json')
    c._require_admission_not_denied(manifest, 'dataset')
    contract = c.load_contract()
    require(manifest.get('schema_id') == c.DATASET_SCHEMA_ID and manifest.get('contract_id') == c.CONTRACT_ID
        and manifest.get('contract_sha256') == c.contract_fingerprint(contract)
        and manifest.get('split_policy') == contract['split_policy'], 'current Common10 dataset contract required')
    c._string(manifest.get('dataset_id'), 'dataset_id')
    references = c._sequence(manifest.get('episodes'), 'episodes')
    require(references, 'dataset requires episodes')
    claims, seen_episodes, seen_samples, seen_sources, rows, episodes, tests = {}, set(), set(), set(), [], [], []
    for ref in references:
        path = c._safe_file(root, ref.get('manifest'), 'episode manifest')
        episode = read(path, ref.get('sha256'))
        c._require_admission_not_denied(episode, 'episode')
        require(episode.get('schema_id') == c.EPISODE_SCHEMA_ID and episode.get('domain') == 'carla',
            'this audit is limited to the current CARLA Common10 data scope')
        eid = c._string(episode.get('episode_id'), 'episode_id')
        require(eid == ref.get('episode_id') and eid not in seen_episodes, 'duplicate or mismatched episode')
        seen_episodes.add(eid)
        split = episode.get('split'); require(split in ('train', 'val', 'test'), 'unsupported split')
        count = c._integer(episode.get('sample_count'), 'sample_count', 1)
        for field in ('site_id', 'map_id', 'route_id', 'weather', 'source_session_id', 'split_group_id', 'scene_group_id'):
            c._string(episode.get(field), field)
        group = episode['split_group_id']
        require(re.fullmatch(re.escape('carla:' + episode['map_id'] + ':') + '[0-9a-f]{64}', group), 'invalid CARLA route group')
        seed = c._integer(episode.get('traffic_seed'), 'traffic_seed', 0)
        require(episode['scene_group_id'] == f"{group}:traffic={seed}:weather={episode['weather']}", 'scene identity differs')
        for field in ('episode_id', 'source_session_id', 'split_group_id', 'scene_group_id'):
            c._claim_split(claims, field, episode[field], split)
        sample_pin = c._sha256(episode.get('sample_jsonl_sha256'), 'samples SHA')
        if split == 'test':
            # HH_260906 - Do not resolve or open TEST samples, source frames, collection config, route or image payloads.
            tests.append(dict(episode_id=eid, declared_sample_count=count, episode_manifest_sha256=ref['sha256'],
                declared_sample_jsonl_sha256=sample_pin, sample_data_opened=False))
            continue
        route_path = c._safe_file(path.parent, episode.get('route_geometry_file'), 'route geometry')
        read(route_path, episode.get('route_geometry_sha256'))
        semantic = c._route_geometry_semantic_hash(route_path, episode=episode, contract=contract,
            expected_sha256=episode['route_geometry_sha256'])
        require(group == f"carla:{episode['map_id']}:{semantic}", 'declared route group differs from semantic geometry')
        provenance = c._mapping(episode.get('source_provenance'), 'source_provenance')
        source_path = c._safe_file(path.parent, provenance.get('source_manifest_file'), 'source manifest')
        source = read(source_path, provenance.get('source_manifest_sha256'))
        c._require_admission_not_denied(source, 'source manifest')
        bound_source = c._validate_source_manifest(source, episode=episode,
            route_geometry_sha256=episode['route_geometry_sha256'], path=source_path)
        metadata = data._episode_metadata(episode, source, provenance['source_manifest_sha256'], 'episode metadata')
        tags = c._sequence(episode.get('scenario_tags'), 'scenario_tags')
        require(tags and all(isinstance(t, str) and t.strip() for t in tags) and len(set(tags)) == len(tags),
            'scenario tags must be unique nonempty strings')
        config_path = c._safe_file(path.parent, provenance.get('collection_config_file'), 'collection config')
        config = read(config_path, provenance.get('collection_config_sha256'))
        c._require_admission_not_denied(config, 'collection config', include_native_metadata=True)
        windows = phase_windows(config)
        sample_path = c._safe_file(path.parent, episode.get('sample_jsonl'), 'selected sample JSONL')
        selected, prior_stamp = [], -1
        start = c._integer(episode.get('start_timestamp_ns'), 'episode start', 0)
        end = c._integer(episode.get('end_timestamp_ns'), 'episode end', 0)
        require(end >= start, 'episode time bounds reverse')
        for _, sample in c._iter_jsonl(sample_path, expected_sha256=sample_pin):
            require(sample.get('schema_id') == c.SAMPLE_SCHEMA_ID and sample.get('episode_id') == eid,
                'sample schema or episode differs')
            token = c._string(sample.get('sample_id'), 'sample_id')
            require(token not in seen_samples and c._integer(sample.get('sequence_index'), 'sequence', 0) == len(selected),
                'duplicate sample or noncontiguous sequence')
            seen_samples.add(token)
            stamp = c._integer(sample.get('anchor_timestamp_ns'), 'anchor', 0)
            require(start <= stamp <= end and stamp > prior_stamp, 'anchor time outside bounds or not increasing')
            prior_stamp = stamp
            source_id = c._string(sample.get('source_sample_id'), 'source_sample_id')
            require(source_id not in seen_sources, 'source sample reused across selected episodes')
            seen_sources.add(source_id)
            c._claim_split(claims, 'source_sample_id', source_id, split)
            bound = bound_source['selected_samples'].get(source_id)
            require(bound is not None and bound['anchor_timestamp_ns'] == stamp, 'sample source identity differs')
            camera_ids = tuple(v.get('source_frame_id') for v in c._sequence(sample.get('camera_bundle'), 'camera bundle'))
            require(camera_ids == bound['camera_source_frame_ids'], 'source camera selection differs')
            navigation = c._mapping(sample.get('navigation'), 'navigation')
            command = c._integer(navigation.get('command'), 'navigation command', 0)
            require(command <= 5, 'navigation command out of range')
            command_stamp = c._integer(navigation.get('command_timestamp_ns'), 'command timestamp', 0)
            require(command_stamp <= stamp and (stamp - command_stamp) * 1e-6 <= contract['capture']['maximum_command_age_ms'],
                'current navigation command must be causal and fresh')
            behavior, valid, labels = planning_group(sample, episode)
            phase = next((name for name, first, last in windows if first <= stamp <= last), 'NOT_RECORDED')
            selected.append(dict(episode_id=eid, split=split, split_group_id=group, site_id=episode['site_id'],
                weather=episode['weather'], motion_group=behavior, capture_phase=phase, command=command,
                valid_points=valid, labels=labels))
        require(len(selected) == count and len(bound_source['selected_samples']) == count, 'selected sample denominator mismatch')
        inputs[sample_path.relative_to(root).as_posix()] = sample_pin
        rows.extend(selected)
        episodes.append(dict(episode_id=eid, split=split, split_group_id=group, map_id=metadata.map_id,
            site_id=metadata.site_id, route_id=metadata.route_id, weather=episode['weather'],
            declared_scenario_tags=list(metadata.scenario_tags), source_manifest_sha256=metadata.source_manifest_sha256,
            coverage=summarize_rows(selected)))
    split_reports = {split: summarize_rows([r for r in rows if r['split'] == split]) for split in ('train', 'val')}
    gaps = []
    for split, report in split_reports.items():
        for group in ('stationary_hold', 'moving_to_stop'):
            if report['motion_groups'][group]['anchor_count'] == 0:
                gaps.append(dict(kind='MISSING_OBSERVED_MOTION_GROUP', split=split, motion_group=group))
        for tag in ('straight', 'left', 'right'):
            matched = [e for e in episodes if e['split'] == split and tag in e['declared_scenario_tags']]
            if not matched:
                gaps.append(dict(kind='MISSING_DECLARED_SCENARIO_EPISODE', split=split, scenario=tag,
                    observed_maneuver_success_proven=False))
    for task in TASKS:
        gaps.append(dict(kind='NO_SUPPORTED_TASK_GROUND_TRUTH', task=task,
            status='NOT_RECORDED' if all(r['labels'][task] == 'NOT_RECORDED' for r in rows) else 'DECLARED_UNAVAILABLE'))
    for name, pin in inputs.items():
        require(sha_file(c._safe_file(root, name, 'postcheck input')) == pin, 'input changed during readiness audit')
    require(source_identity() == sources, 'readiness source changed during audit')
    metadata_fingerprint = hashlib.sha256(json.dumps(tests, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return dict(schema=SCHEMA, status='AUDITED_NOT_TRAINING_APPROVED', created_at_utc=datetime.now(timezone.utc).isoformat(),
        dataset_id=manifest['dataset_id'], dataset_manifest_sha256=inputs['dataset.json'], source_sha256=sources,
        input_sha256=inputs, splits=split_reports, episodes=episodes,
        route_groups={split: {g: summarize_rows([r for r in rows if r['split'] == split and r['split_group_id'] == g])
            for g in sorted({r['split_group_id'] for r in rows if r['split'] == split})} for split in ('train', 'val')},
        test_metadata=dict(episode_count=len(tests), declared_sample_count=sum(t['declared_sample_count'] for t in tests),
            metadata_fingerprint_sha256=metadata_fingerprint, episodes=tests, target_behavior='NOT_OPENED'),
        coverage_gaps=gaps, training_data_approved=False, automatic_promotion=False, vehicle_control_approved=False,
        scope=dict(train_val_targets_opened=bool(rows), test_targets_opened=False, test_routes_opened=False,
            images_opened=False, neural_inference=False, full_dataset_validation=False, labels_modified=False),
        interpretation=['Counts describe overlapping anchors; distinct episodes, route groups, sites and weather are separate denominators.',
            'Scenario tags and current navigation commands do not prove an observed turn or a successful maneuver.',
            'Capture phases use hash-bound native phase intervals; absent metadata is NOT_RECORDED, not driving or stopped.',
            'Absent task ground truth stays unknown; an unavailable task is not an empty-scene negative label.',
            'TEST route-group claims are checked for metadata collisions only; TEST geometry and targets remain unopened.',
            'This limited coverage audit does not certify capture quality, sensor validity, physical feasibility, split independence beyond checked identities or admission.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    output, dataset = nonsymlink(args.output), nonsymlink(args.dataset)
    require(not output.exists() and not output.resolve().is_relative_to(dataset.resolve()),
        'output must be fresh and outside the immutable dataset')
    report = audit_dataset(dataset)
    output.parent.mkdir(parents=True, exist_ok=True)
    nonsymlink(output)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(status=report['status'], train=report['splits']['train']['anchor_count'],
        val=report['splits']['val']['anchor_count'], coverage_gap_count=len(report['coverage_gaps']))))


if __name__ == '__main__':
    main()
