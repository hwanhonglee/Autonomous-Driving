#!/usr/bin/env python3
"""HH_260906 - Qualify one immutable Town03 DEV moving task for explicit Common10 conversion, not model release."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.e2e import prepare_carla_common10_dev_v2 as adapter
from scripts.e2e import carla_moving_capture_dev_v2 as moving
from portable_e2e import contract, dataset

CASE = 'artifacts/training/2026-09-16/moving_dev_capture_v2/01_town03_straight'
PREPARED = 'artifacts/training/2026-09-16/moving_dev_capture_v2/prepared_moving_dev_v2'
PLAN_SHA = 'ccf89eb0a77c2a46d109d026fdfc68ac1ee1685306ce3722b78e9972d3846e37'
OWNER_SHA = '54c76359bff26daae5c65585a7a6a76008ab8e15ee38692e6d2bc1748df9abf3'
AUDIT_SHA = 'a7f1faff85a88a588d4ec8c49dc7b29e1838f179f845ea3908c5c5d67768549b'
COMPANION_SHA = {
    'jpeg_audit.jsonl': '623dcde9a2ac62827601ccf1bd8b2c1b82f9391bf36a4bbbbd9f22f2a1ab12ba',
    'future_anchor_audit.jsonl': '70d89a2be02d9e3545bbe2b276b7995c901258e1cc7ddaf66d45554c9a3f2304',
    'native_projection.jsonl': 'de545a03a28ab5db51e698d32ad5081b1178ea63b3208cbd3d69de21657647b3',
}
SCHEMA = 'carla.moving_dev_common10_conversion_receipt.v2'
SOURCE_PATHS = (
    'scripts/e2e/qualify_carla_moving_dev_common10_v2.py', 'scripts/e2e/prepare_carla_common10_dev_v2.py',
    'scripts/e2e/prepare_carla_common10_dataset.py', 'scripts/e2e/carla_moving_capture_dev_v2.py',
    'scripts/e2e/carla_moving_capture.py', 'scripts/e2e/audit_portable_target_feasibility.py',
    'portable_e2e/__init__.py', 'portable_e2e/contract.py', 'portable_e2e/dataset.py',
    'portable_e2e/model.py', 'portable_e2e/runtime_contract.py',
    'portable_e2e/stop_primitive_research.py',
    'portable_e2e/config/common_10hz_v1.contract.json',
)


def require(condition, message):
    if not condition:
        raise adapter.AdapterError(message)


def safe(path):
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve()
            and not any(p.is_symlink() for p in (path, *path.parents)), 'qualification path must be canonical and nonsymlink')
    require(path.is_relative_to(ROOT), 'qualification path escapes repository')
    return path


def sha(path):
    return adapter._sha256(safe(path))


def hash_value(value):
    require(isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value),
            'expected external SHA256 must be lowercase 64-hex')
    return value


def document(path):
    value = adapter._json(safe(path))
    json.dumps(value, allow_nan=False)
    return value


def companion_pins(audit_path):
    # HH_260906 - These historical JPEG/future journals require independent pins, not self-asserted current hashes.
    result = {}
    for name, expected in COMPANION_SHA.items():
        path = safe(audit_path.parent / name)
        require(sha(path) == expected, 'historical diagnostic companion changed: ' + name)
        result[path.relative_to(ROOT).as_posix()] = expected
    return result


def check_denials(manifest):
    # HH_260906 - Existing admission-bearing containers include initialization; no old denial is overridden.
    adapter._require_native_admission_not_denied(manifest)
    contract._require_admission_not_denied(manifest, 'native manifest', include_native_metadata=True)


def scope_manifest(manifest):
    check_denials(manifest)
    result = manifest.get('result', {})
    require(manifest.get('status') == 'complete' and manifest.get('cleanup', {}).get('completed') is True,
            'moving native capture must be complete with cleanup')
    require(result.get('task_complete') is True and result.get('goal_reached') is False,
            'qualification requires the truthful complete-moving/false-catalog-goal task')
    for obj in (result, result.get('moving_task', {}), manifest.get('capture_contract', {}).get('agent_initialization', {})):
        require(obj.get('admission_status') == 'PENDING_QA' and obj.get('dataset_conversion_performed') is False
                and 'training_data_approved' not in obj and 'development_only' not in obj,
                'only the fresh pending moving capture may qualify')
    require(result.get('collision_event_count') == result.get('lane_invasion_event_count') == 0, 'moving capture contains events')
    require(manifest['capture_contract']['moving_task']['plan_sha256'] == PLAN_SHA, 'wrong moving task plan')
    capture = manifest['capture_contract']['moving_task']
    fixed = moving.plan_contract()
    require(all(key in capture for key in fixed)
            and moving.encoded({key: capture[key] for key in fixed}) == moving.encoded(fixed),
            'only the exact Town03 DEV v2 native contract may qualify')
    require(capture['dataset_split_intent'] == 'val', 'moving DEV qualification requires VAL intent')


def selected_replay(case, manifest):
    episode = case / 'episode'
    states = adapter._jsonl(episode / 'states.jsonl')
    cameras = adapter._jsonl(episode / 'camera_frames.jsonl')
    route = document(episode / 'route.json')
    plan = document(episode / 'moving_task_plan.json')
    require(sha(episode / 'moving_task_plan.json') == PLAN_SHA, 'moving plan bytes changed')
    require(sha(episode / 'route.json') == moving.ROUTE_SHA, 'moving route bytes changed')
    replay = moving.reconstruct(plan, states, cameras, route['route'])
    phases = Counter(row['capture_phase'] for row in states)
    require(phases == dict(stationary_warmup=70, driving=951, stationary_tail=130), 'exact native denominator changed')
    require(replay.result(tail_states=130, goal_reached=False) == manifest['result']['moving_task'], 'moving task replay differs')
    require(replay.failure is None and len(cameras) == 576 and len(replay.selected) == 328, 'fixed moving denominator changed')
    require([x['frame'] for x in replay.selected] == list(range(1261, 1916, 2)), 'fixed selected frame order changed')
    require(replay.last['timestamp_ns'] - replay.first['timestamp_ns'] == 32_700_000_487, 'selected observed span changed')
    for camera in cameras:
        stamps = [round(value * 1e9) for value in camera['source_timestamps'].values()]
        require(len(set(stamps)) == 1, 'this exact case requires median and latest-sensor anchor clocks to agree')
    return states, cameras, replay.selected


def direct_route_identity(case, manifest, route, owner):
    # HH_260906 - This pinned catalog is already in its recorded coordinates; never invent coordinate_alignment.
    require(sha(case / 'owner_plan.json') == OWNER_SHA, 'moving route identity owner pin differs')
    item = owner['moving_task_capture']['inputs']['route_file']
    expected_path = str(ROOT / moving.ROUTE_PATH)
    require(owner['route_path'] == item['path'] == manifest['provenance']['source_route'] == expected_path,
            'moving direct catalog route source paths differ')
    require(owner['route_sha256'] == item['sha256'] == manifest['provenance']['route_sha256']
            == sha(case / 'episode/route.json') == moving.ROUTE_SHA, 'moving direct catalog route source hashes differ')
    require(document(case / 'episode/route.json') == route and route.get('town') == 'Town03'
            and route.get('scenario') == 'straight' and route.get('coordinate_reference') == 'base_link'
            and 'coordinate_alignment' not in route, 'moving direct catalog route representation differs')
    route_id = Path(item['path']).stem
    require(route_id == 'town03_straight_s0000_p07', 'moving route identity differs')
    return dict(route_id=route_id, source_route_path=item['path'], source_route_sha256=item['sha256'],
        identity_source='authenticated_owner_route_input_and_unchanged_native_catalog_artifact',
        native_coordinate_reference=route['coordinate_reference'], coordinate_alignment_synthesized=False,
        native_route_alignment_transform_performed=False,
        notice='Direct catalog world-space route retained byte-for-byte. Per-sample Common10 map-to-base navigation projection remains unchanged.')


def native_route_id(spec, manifest, route, qualification):
    verify_native_scope(spec, manifest, qualification)
    identity = direct_route_identity(spec.path.parent, manifest, route, document(spec.path.parent / 'owner_plan.json'))
    require(identity == qualification['document']['route_identity'], 'moving route identity differs from conversion receipt')
    return identity['route_id']


def evidence(case_root, raw_audit, raw_audit_sha256, *, decode_images):
    case, audit_path = safe(case_root), safe(raw_audit)
    source_pins = {name: sha(ROOT / name) for name in SOURCE_PATHS}
    require(case == ROOT / CASE, 'qualification is restricted to the exact fresh moving case')
    require(hash_value(raw_audit_sha256) == AUDIT_SHA and sha(audit_path) == AUDIT_SHA, 'independently pinned raw audit differs')
    manifest_path = case / 'episode/manifest.json'
    manifest = document(manifest_path)
    scope_manifest(manifest)  # HH_260906 - Denial checks precede any native rows or image payload read.
    audit = document(audit_path)
    require(audit['schema'] == 'carla.moving_capture_diagnostic.v2' and audit['status'] == 'DIAGNOSED_NOT_ADMITTED'
            and audit['moving_scope_necessary_checks_clear'] is True, 'raw moving necessary checks are not clear')
    require(audit['trial'] == str(ROOT / CASE) and audit['owner_plan_sha256'] == OWNER_SHA, 'wrong raw diagnostic owner')
    selection = dict(path=moving.SELECTION_PATH, sha256=moving.SELECTION_SHA)
    require(audit['dataset_split_intent'] == 'val' and audit['route_selection_evidence'] == selection,
            'DEV diagnostic split or route-selection evidence differs')
    pins = {}

    def pin(path, expected=None):
        path = safe(path)
        digest = sha(path)
        require(expected is None or digest == expected, 'qualification input changed: ' + str(path))
        key = path.relative_to(ROOT).as_posix()
        require(key not in pins or pins[key] == digest, 'qualification input changed between repeated reads: ' + key)
        pins[key] = digest
        return digest

    pin(audit_path, AUDIT_SHA)
    for item in audit['source_manifest']:
        pin(case / item['path'], item['sha256'])
    pin(case / 'owner_plan.json', OWNER_SHA)
    owner = document(case / 'owner_plan.json')
    owner_result = document(case / 'owner_result.json')
    pin(case / 'owner_started.json')
    require(owner_result['exit_code'] == 0 and owner_result['moving_task_capture']['capture_complete_pending_qa'] is True,
            'owned capture did not complete')
    pin(manifest_path, owner_result['moving_task_capture']['final_manifest_sha256'])
    for name, expected in owner['source_sha256'].items():
        pin(case / 'provenance' / name, expected)
        pin(ROOT / name, expected)
    refs = owner['moving_task_capture']
    selection_ref = refs['route_selection_evidence']
    require(refs['schema'] == 'carla.moving_task_capture_plan.v2'
            and selection_ref == dict(path=str(ROOT / moving.SELECTION_PATH), sha256=moving.SELECTION_SHA,
                archived_path='provenance/route_selection_evidence.json')
            and manifest['capture_contract']['moving_task']['route_selection_evidence'] == selection,
            'DEV owner route-selection evidence differs')
    for item in [refs['plan'], *refs['inputs'].values(), selection_ref]:
        pin(Path(item['path']), item['sha256'])
        pin(case / item['archived_path'], item['sha256'])
    for name, expected in audit['diagnostic_source']['files'].items():
        pin(audit_path.parent / 'source_archive' / name, expected)
        # HH_260906 - The separate DEV adapter preserves every original diagnostic and label-math source pin.
        pin(ROOT / name, expected)
    for name, expected in companion_pins(audit_path).items():
        pin(ROOT / name, expected)
    states, cameras, selected = selected_replay(case, manifest)
    route_identity = direct_route_identity(case, manifest, document(case / 'episode/route.json'), owner)
    images = adapter._jsonl(audit_path.parent / 'jpeg_audit.jsonl')
    expected_paths = {path for camera in cameras for path in camera['images'].values()}
    require(len(images) == len(expected_paths) == 3456 and {r['path'] for r in images} == expected_paths,
            'raw image audit denominator differs')
    actual_paths = {p.relative_to(case / 'episode').as_posix() for p in (case / 'episode/images').rglob('*') if p.is_file()}
    require(actual_paths == expected_paths, 'unreferenced or missing raw image payload')
    if decode_images:
        from PIL import Image
    for image in images:
        path = case / 'episode' / image['path']
        pin(path, image['sha256'])
        require(image['decoded'] is True and image['referenced'] is True and image['expected_size'] == [640, 360],
                'raw image evidence is incomplete')
        if decode_images:
            with Image.open(path) as opened:
                opened.load()
                require(opened.size == (640, 360) and opened.mode == image['mode']
                        and hashlib.sha256(opened.tobytes()).hexdigest() == image['decoded_pixel_sha256'],
                        'original decoded image bytes differ')
    require(audit['selected_future']['all_selected_anchors_have_actual_64_points'] is True
            and set(audit['selected_future']['unchanged_nine_metric_failures'].values()) == {0}, 'selected raw targets failed')
    for name, expected in (pins | source_pins).items():
        require(sha(ROOT / name) == expected, 'qualification input changed during inspection')
    return dict(schema=SCHEMA, status='QUALIFIED_FOR_COMMON10_CONVERSION', case=CASE,
        dataset_split='val', route_selection_evidence=selection,
        plan_sha256=PLAN_SHA, owner_plan_sha256=OWNER_SHA, raw_audit_path=audit_path.relative_to(ROOT).as_posix(),
        raw_audit_sha256=AUDIT_SHA, source_sha256=source_pins, input_sha256=pins,
        native_manifest_sha256=sha(manifest_path), route_identity=route_identity, selected_camera_frames=[x['frame'] for x in selected],
        selected_anchors=selected, raw_camera_anchors=576, context_only_camera_anchors=248,
        selected_camera_anchors=328, raw_images=3456, selected_images=1968, selected_span_ns=32_700_000_487,
        full_raw_failures_retained=audit['full_raw_future_all_original_phases'],
        conversion_only=True, prepared_dataset_validation_performed=False, training_run_performed=False,
        catalog_goal_reached=False, model_or_vehicle_release_approved=False,
        notice='This receipt authorizes one VAL-only staged conversion. Final qualification still requires unchanged Common10 validation, VAL loading and exact native-to-prepared reconstruction.')


def recheck(qualification):
    require(sha(qualification['receipt_path']) == qualification['receipt_sha256'], 'qualification receipt changed')
    doc = qualification['document']
    for name, expected in (doc['input_sha256'] | doc['source_sha256']).items():
        require(sha(ROOT / name) == expected, 'qualification source/input changed: ' + name)


def authorize(args):
    path = getattr(args, 'moving_qualification', None)
    expected = getattr(args, 'moving_qualification_sha256', None)
    require(path is not None and expected is not None, 'paired moving qualification path and external SHA required')
    path = safe(path)
    require(sha(path) == hash_value(expected), 'qualification receipt external SHA differs')
    require(len(args.episode) == 1 and args.episode[0].split == 'val'
            and safe(args.episode[0].path) == ROOT / CASE / 'episode', 'moving conversion permits one exact VAL input only')
    require(args.image_mode == 'copy' and args.validate_mode == 'planning', 'moving conversion requires copy/planning mode')
    output = safe(args.output)
    require(output == ROOT / PREPARED, 'moving conversion requires the fixed fresh prepared output')
    for protected in (ROOT / CASE, ROOT / (CASE + '_analysis'), path):
        require(not output.is_relative_to(protected) and not protected.is_relative_to(output), 'moving output overlaps protected evidence')
    doc = document(path)
    require(doc.get('schema') == SCHEMA and doc.get('case') == CASE and doc.get('dataset_split') == 'val',
            'moving DEV conversion requires its exact VAL receipt')
    protected_audit = safe(ROOT / doc['raw_audit_path']).parent
    for target in (output, output.with_name(output.name + '.partial')):
        require(not target.is_relative_to(protected_audit) and not protected_audit.is_relative_to(target),
                'moving output overlaps the authenticated raw audit')
    check_denials(document(ROOT / CASE / 'episode/manifest.json'))
    expected_doc = evidence(ROOT / CASE, ROOT / doc['raw_audit_path'], doc['raw_audit_sha256'], decode_images=False)
    require(doc == expected_doc, 'qualification receipt does not match independently reconstructed evidence')
    result = dict(receipt_path=str(path), receipt_sha256=expected, document=doc)
    recheck(result)
    return result


def verify_native_scope(spec, manifest, qualification):
    scope_manifest(manifest)
    require(spec.split == 'val' and safe(spec.path) == ROOT / CASE / 'episode', 'wrong moving native input')
    require(qualification['document']['native_manifest_sha256'] == sha(spec.path / 'manifest.json'), 'native manifest binding differs')
    require(qualification['document']['schema'] == SCHEMA and qualification['document']['dataset_split'] == 'val'
            and qualification['document']['plan_sha256'] == PLAN_SHA and qualification['document']['conversion_only'] is True,
            'moving conversion receipt scope differs')


def independently_expected(native, sample, state_by_stamp, stamps, polyline, route_hash, contract_data):
    # HH_260906 - Independently reconstruct labels from immutable native measurements, without the adapter interpolation helpers.
    anchor = sample['anchor_timestamp_ns']
    ego_stamp = stamps[bisect_right(stamps, anchor) - 1]
    ego = state_by_stamp[ego_stamp]
    yaw = ego['yaw']; cosine, sine = math.cos(yaw), math.sin(yaw)
    positions, yaws, speeds, times = [], [], [], []
    for index in range(64):
        stamp = anchor + (index + 1) * 100_000_000
        upper = bisect_right(stamps, stamp)
        require(upper > 0, 'target precedes raw native support')
        left_stamp = stamps[upper - 1]; left = state_by_stamp[left_stamp]
        if left_stamp == stamp:
            future = {key: float(left[key]) for key in ('x', 'y', 'yaw', 'vx', 'vy')}
        else:
            require(upper < len(stamps), 'target lacks actual raw future')
            right_stamp = stamps[upper]; right = state_by_stamp[right_stamp]
            ratio = (stamp - left_stamp) / (right_stamp - left_stamp)
            future = {key: float(left[key]) + ratio * (float(right[key]) - float(left[key])) for key in ('x', 'y', 'vx', 'vy')}
            delta = math.atan2(math.sin(float(right['yaw']) - float(left['yaw'])), math.cos(float(right['yaw']) - float(left['yaw'])))
            interpolated = float(left['yaw']) + ratio * delta
            future['yaw'] = math.atan2(math.sin(interpolated), math.cos(interpolated))
        dx, dy = future['x'] - float(ego['x']), future['y'] - float(ego['y'])
        positions.append([cosine * dx + sine * dy, -sine * dx + cosine * dy])
        yaws.append(math.atan2(math.sin(future['yaw'] - yaw), math.cos(future['yaw'] - yaw)))
        speeds.append(math.hypot(future['vx'], future['vy'])); times.append(stamp)
    quaternion = [0., 0., math.sin(.5 * yaw), math.cos(.5 * yaw)]
    route, goal, arc = contract._canonical_route_in_base(tuple(tuple(p) for p in polyline),
        position_m=(ego['x'], ego['y'], ego['z']), orientation_xyzw=quaternion, contract=contract_data, context='independent moving reconstruction')
    expected_ego = dict(timestamp_ns=ego_stamp, frame_id='map', child_frame_id='base_link',
        position_m=[ego['x'], ego['y'], ego['z']], orientation_xyzw=quaternion,
        linear_velocity_base_mps=[ego['vx'], ego['vy'], 0.], angular_velocity_base_radps=[0., 0., ego['yaw_rate']],
        linear_acceleration_base_mps2=[ego['ax'], ego['ay'], 0.], steering_tire_angle_rad=ego['steering_tire_angle_rad'])
    expected_navigation = dict(route_id=native.route_id, route_source='episode_route_geometry',
        route_source_timestamp_ns=stamps[0], route_geometry_sha256=route_hash, route_anchor_arc_m=arc,
        command=ego['command'], command_timestamp_ns=ego_stamp, speed_limit_mps=ego['speed_limit_mps'],
        route_polyline_base_m=[list(p) for p in route], goal_base_m=list(goal))
    expected_planning = dict(available=True, dt_s=.1, positions_base_xy_m=positions, yaw_rad=yaws, speed_mps=speeds,
        valid=[True] * 64, target_timestamp_ns=times, invalid_reason=[None] * 64)
    require(sample['ego'] == expected_ego and sample['navigation'] == expected_navigation
            and sample['labels']['planning'] == expected_planning, 'native-to-prepared ego/navigation/64-target reconstruction differs')


def validate_prepared(prepared, native, qualification, native_report):
    from scripts.e2e import audit_portable_target_feasibility as targets
    recheck(qualification)
    prepared = safe(prepared)
    dataset_doc = document(prepared / 'dataset.json')
    require(len(dataset_doc['episodes']) == 1, 'DEV prepared dataset requires one exact episode')
    episode_path = prepared / dataset_doc['episodes'][0]['manifest']; episode_root = episode_path.parent
    episode = document(episode_path)
    require(episode['split'] == 'val' and episode['source_provenance']['adapter_id'] == 'carla_fixed_moving_dev_task_to_common10_v2',
            'wrong prepared split or adapter identity')
    loaded = dataset.load_training_examples(prepared, split='val', mode='planning', check_image_hashes=True)
    require(loaded.split == 'val' and len(loaded.examples) == 328
            and native_report['sample_count'] == 328 and native_report['episode_count'] == 1,
            'unchanged validator/loader did not produce exactly 328 VAL examples')
    empty_splits = verify_empty_splits(prepared)
    samples = adapter._jsonl(episode_root / 'samples.jsonl')
    collection = document(episode_root / 'collection_config.json')
    require(collection['adapter_id'] == 'carla_fixed_moving_dev_task_to_common10_v2', 'wrong DEV collection adapter identity')
    require(collection['native_capture_contract'] == native.manifest['capture_contract']
            and collection['native_runtime'] == native.manifest['runtime'] and collection['native_result'] == native.manifest['result'],
            'native capture/runtime/result metadata changed')
    require(sha(episode_root / 'native_manifest.json') == native.source_hashes['manifest.json']
            and sha(episode_root / 'native_route.json') == native.source_hashes['route.json']
            and sha(episode_root / 'moving_qualification.json') == qualification['receipt_sha256'], 'raw provenance copy differs')
    require(collection['moving_qualification']['route_identity'] == qualification['document']['route_identity'],
            'prepared direct route identity differs')
    timeline = adapter._state_timeline(native)
    state_by_stamp = dict(timeline); stamps = list(state_by_stamp)
    polyline = []
    for point in native.route['route']:
        xy = [float(point['x']), float(point['y'])]
        if not polyline or math.dist(polyline[-1], xy) > 1e-9:
            polyline.append(xy)
    require(document(episode_root / 'route_geometry.json') == dict(route_id=native.route_id, frame_id='map', polyline_m=polyline),
            'prepared route geometry differs from native route')
    source_manifest = document(episode_root / 'source_manifest.json')
    raw_frames = source_manifest['camera_frames']
    require(len(raw_frames) == 3456, 'full raw image provenance was lost')
    for image in raw_frames:
        expected = qualification['document']['input_sha256'][CASE + '/episode/' + image['source_uri']]
        require(image['source_payload_sha256'] == expected, 'raw context image digest changed')
    by_frame = {r['frame']: r for r in native.camera_frames}
    audits = []; copied = []
    require(len(samples) == len(qualification['document']['selected_anchors']) == 328, 'selected sample count changed')
    contract_data = contract.load_contract()
    for index, (sample, selected) in enumerate(zip(samples, qualification['document']['selected_anchors'])):
        require(sample['sequence_index'] == index and sample['anchor_timestamp_ns'] == selected['timestamp_ns'], 'selected ordering/timestamp changed')
        camera = by_frame[selected['frame']]
        independently_expected(native, sample, state_by_stamp, stamps, polyline, episode['route_geometry_sha256'], contract_data)
        for item, name in zip(sample['camera_bundle'], contract.CAMERA_ORDER):
            source = native.spec.path / camera['images'][name]; destination = episode_root / item['path']
            require(item['name'] == name and item['source_frame_id'] == f"carla:{selected['frame']:010d}:{name}"
                    and sha(source) == sha(destination) == item['sha256'], 'prepared image identity differs')
            require(source.stat().st_ino != destination.stat().st_ino and destination.stat().st_nlink == 1,
                    'prepared images must be independent copies')
            copied.append(destination)
        audited = targets.audit_sample(sample, route_length_m=native.route['route_length_m'],
            episode_start_ns=episode['start_timestamp_ns'], episode_end_ns=episode['end_timestamp_ns'],
            tail_start_ns=next(stamp for stamp, state in timeline if state['capture_phase'] == 'stationary_tail'))
        audited['episode_id'] = native.episode_id; audits.append(audited)
    require(len(copied) == 1968 and set(copied) == {p for p in (episode_root / 'images').rglob('*') if p.is_file()}, 'prepared image denominator differs')
    accounting = episode['capture_accounting']
    require(accounting['raw_anchor_count'] == 576 and accounting['selected_bundle_count'] == 328
            and accounting['ineligible_anchor_reasons'] == {'moving_task_fixed_window_context_only': 248}, 'context-only ledger differs')
    measured = targets.summarize_samples(audits, 64)
    require(measured['valid_step_count'] == 20_992 and measured['invalid_step_count'] == 0
            and all(v['violating_step_count'] == 0 for v in measured['metrics'].values()), 'prepared targets fail unchanged necessary bounds')
    recheck(qualification)
    return dict(schema='carla.moving_dev_common10_conversion_result.v2', status='QUALIFIED_COMMON10_PLANNING_CONVERSION',
        qualified_for_common10_planning_conversion=True, qualification_receipt_sha256=qualification['receipt_sha256'],
        dataset_manifest_sha256=sha(prepared / 'dataset.json'), validation_report_sha256=sha(prepared / 'validation_report.json'),
        val_loader_fingerprint_sha256=loaded.fingerprint_sha256, train_examples=0, val_examples=328, test_examples=0,
        empty_split_loader_checks=empty_splits,
        exact_native_ego_navigation_and_target_reconstruction=True, rederived_target_points=20_992,
        raw_camera_anchors=576, selected_camera_anchors=328, context_only_camera_anchors=248,
        raw_image_hashes_preserved=3456, independently_copied_selected_images=1968, unchanged_target_audit=measured,
        full_raw_failures_retained=qualification['document']['full_raw_failures_retained'],
        native_goal_reached=False, native_pending_metadata_unchanged=True, training_run_performed=False,
        route_identity=qualification['document']['route_identity'], native_route_bytes_unchanged=True,
        model_or_vehicle_release_approved=False, manual_release_review_required=True,
        notice='One fixed Town03 moving DEV task only. Native validation, raw failures and physical-actuation limitations remain unchanged; no training, release-test or all-map release claim.')


def verify_empty_splits(prepared):
    # HH_260906 - The real loader must reject TRAIN and TEST as empty; other validation failures do not prove emptiness.
    checks = {}
    for split in ('train', 'test'):
        try:
            dataset.load_training_examples(prepared, split=split, mode='planning', check_image_hashes=True)
        except contract.ContractError as error:
            require(str(error) == f"dataset split {split!r} contains no samples", 'unexpected empty-split loader failure')
            checks[split] = 'NO_SAMPLES'
        else:
            require(False, 'DEV prepared dataset exposes non-VAL examples: ' + split)
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--case-root', type=Path, required=True)
    parser.add_argument('--raw-audit', type=Path, required=True)
    parser.add_argument('--raw-audit-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    output = safe(args.output)
    require(not output.exists(), 'qualification receipt must be fresh')
    for protected in (safe(args.case_root), safe(args.raw_audit).parent):
        require(not output.is_relative_to(protected) and not protected.is_relative_to(output), 'receipt overlaps original evidence')
    value = evidence(args.case_root, args.raw_audit, args.raw_audit_sha256, decode_images=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        stream.write((json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode())
        stream.flush(); __import__('os').fsync(stream.fileno())
    print(json.dumps(dict(receipt=str(output), sha256=sha(output), status=value['status'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
