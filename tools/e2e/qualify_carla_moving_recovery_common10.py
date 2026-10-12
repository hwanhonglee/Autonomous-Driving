#!/usr/bin/env python3
"""HH_260906 - Qualify one sealed recovery TRAIN capture for staged Common10 conversion only."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from portable_e2e import contract, dataset
from scripts.e2e import audit_carla_moving_capture as moving_audit
from scripts.e2e import carla_moving_capture as moving
from scripts.e2e import prepare_carla_common10_dataset as adapter


BASE = 'artifacts/training/2026-09-15/moving_cohort_v1'
SCHEMA = 'carla.moving_recovery_common10_conversion_receipt.v2'
SOURCE_PATHS = (
    'scripts/e2e/qualify_carla_moving_recovery_common10.py',
    'scripts/e2e/prepare_carla_common10_recovery.py',
    'scripts/e2e/prepare_carla_common10_dataset.py',
    'scripts/e2e/carla_moving_capture.py',
    'scripts/e2e/audit_carla_moving_capture.py',
    'scripts/e2e/audit_portable_target_feasibility.py',
    'portable_e2e/__init__.py',
    'portable_e2e/contract.py',
    'portable_e2e/dataset.py',
    'portable_e2e/model.py',
    'portable_e2e/runtime_contract.py',
    'portable_e2e/stop_primitive_research.py',
    'portable_e2e/config/common_10hz_v1.contract.json',
)
COMPANIONS = ('jpeg_audit.jsonl', 'future_anchor_audit.jsonl', 'native_projection.jsonl')
QUALIFIABLE_SLOT_IDS = ('T07-S02', 'CT-S01', 'CT-S02', 'T01-S01')
_DYNAMIC_PLAN_FIELDS = {'declared_at_utc', 'output', 'input_paths', 'input_sha256', 'source_sha256'}


def require(condition, message):
    """HH_260906 - Raise the adapter's closed-form error when one qualification invariant fails."""
    if not condition:
        raise adapter.AdapterError(message)


def safe(path):
    """HH_260906 - Resolve only canonical nonsymlink paths contained by this repository."""
    path = Path(path).absolute()
    require('..' not in path.parts and path == path.resolve()
            and not any(item.is_symlink() for item in (path, *path.parents)),
            'recovery qualification path must be canonical and nonsymlink')
    require(path.is_relative_to(ROOT), 'recovery qualification path escapes repository')
    return path


def sha(path):
    """HH_260906 - Hash one authenticated repository file after canonical path validation."""
    return adapter._sha256(safe(path))


def hash_value(value, label='external SHA256'):
    """HH_260906 - Accept only an explicitly supplied lowercase SHA-256 value."""
    require(isinstance(value, str) and len(value) == 64
            and all(character in '0123456789abcdef' for character in value),
            f'{label} must be lowercase 64-hex')
    return value


def document(path):
    """HH_260906 - Read one strict finite JSON object from an authenticated path."""
    value = adapter._json(safe(path))
    json.dumps(value, allow_nan=False)
    return value


def jsonl(path):
    """HH_260906 - Read one strict nonempty JSON-lines journal from an authenticated path."""
    return adapter._jsonl(safe(path))


def encoded(value):
    """HH_260906 - Produce deterministic finite bytes for semantic replay comparisons."""
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def check_denials(manifest):
    """HH_260906 - Preserve every existing native admission denial before payload inspection."""
    adapter._require_native_admission_not_denied(manifest)
    contract._require_admission_not_denied(manifest, 'native manifest', include_native_metadata=True)


def slot_case(slot_id):
    """HH_260906 - Derive the sole permitted raw case path from the sealed recovery slot registry."""
    require(slot_id in QUALIFIABLE_SLOT_IDS,
            'only one successfully captured and audited recovery TRAIN straight slot may qualify')
    spec = moving.slot_spec(slot_id)
    return safe(ROOT / spec['capture_path']).parent


def prepared_path(slot_id):
    """HH_260906 - Derive a slot-specific fresh output so separate captures can never overwrite each other."""
    token = slot_id.lower().replace('-', '_')
    return safe(ROOT / BASE / f'prepared_recovery_{token}_common10_v1')


def resolve_scope(case):
    """HH_260906 - Bind a raw case to exactly one unchanged five-slot capture contract."""
    case = safe(case)
    plan_path = case / 'episode/moving_task_plan.json'
    plan = document(plan_path)
    slot_id = moving.slot_id_from_plan(plan)
    require(slot_id in QUALIFIABLE_SLOT_IDS,
            'legacy, failed, partial, or unsealed moving slot is outside recovery qualification')
    moving.verify_cohort_slot(slot_id)
    spec = moving.slot_spec(slot_id)
    fixed = moving.plan_contract(slot_id, 15.)
    require(case == slot_case(slot_id), 'raw case path differs from the sealed recovery slot')
    require(set(plan) == set(fixed) | _DYNAMIC_PLAN_FIELDS, 'moving plan fields differ from sealed contract')
    require(moving.encoded({key: plan[key] for key in fixed}) == moving.encoded(fixed),
            'fixed moving recovery plan contract changed')
    require(plan['output'] == spec['capture_path'] and plan['input_paths']['route_file'] == spec['route_path']
            and plan['input_sha256']['route_file'] == spec['route_sha256'],
            'moving plan route or output differs from sealed slot')
    return dict(slot_id=slot_id, case=case.relative_to(ROOT).as_posix(), town=spec['town'],
                route_id=spec['route_id'], route_path=spec['route_path'],
                route_sha256=spec['route_sha256'], plan_sha256=sha(plan_path), plan=plan)


def scope_manifest(manifest, scope):
    """HH_260906 - Admit only completed collision-free pending-QA bytes for the resolved slot."""
    check_denials(manifest)
    result = manifest.get('result', {})
    require(manifest.get('status') == 'complete' and manifest.get('cleanup', {}).get('completed') is True,
            'recovery native capture must be complete with cleanup')
    require(result.get('task_complete') is True and result.get('goal_reached') is False,
            'recovery qualification requires truthful moving completion without catalog-goal success')
    for value in (result, result.get('moving_task', {}),
                  manifest.get('capture_contract', {}).get('agent_initialization', {})):
        require(value.get('admission_status') == 'PENDING_QA'
                and value.get('dataset_conversion_performed') is False
                and 'training_data_approved' not in value and 'development_only' not in value,
                'only unchanged pending-QA native metadata may qualify')
    require(result.get('collision_event_count') == result.get('lane_invasion_event_count') == 0,
            'recovery capture contains collision or lane-invasion events')
    capture = manifest.get('capture_contract', {}).get('moving_task', {})
    require(capture.get('plan_sha256') == scope['plan_sha256']
            and capture.get('cohort_slot_id') == scope['slot_id']
            and capture.get('dataset_split_intent') == 'train',
            'native manifest is not bound to the sealed recovery plan')


def pin_source_manifest(case, audit, pin):
    """HH_260906 - Rehash every audit-ledger file and reject omissions, duplicates, traversal, or size changes."""
    rows = audit.get('source_manifest')
    require(isinstance(rows, list) and rows, 'raw audit source manifest is absent')
    required = {'episode/manifest.json', 'episode/route.json', 'episode/states.jsonl',
                'episode/camera_frames.jsonl', 'episode/moving_task_plan.json',
                'owner_plan.json', 'owner_result.json'}
    seen = set()
    for row in rows:
        require(isinstance(row, dict) and set(row) == {'path', 'sha256', 'size_bytes'},
                'raw audit source manifest row differs')
        name = row['path']
        require(isinstance(name, str) and name and name not in seen, 'duplicate or invalid raw source path')
        seen.add(name)
        path = safe(case / name)
        require(path.is_relative_to(case) and path.is_file(), 'raw source path escapes case or is missing')
        require(path.stat().st_size == row['size_bytes'], 'raw source size changed: ' + name)
        pin(path, hash_value(row['sha256'], 'raw source SHA256'))
    require(required <= seen, 'raw audit source manifest omits qualification-critical files')


def direct_route_identity(case, manifest, route, owner, scope, owner_sha256):
    """HH_260906 - Authenticate the unchanged catalog route without synthesizing coordinate alignment."""
    require(sha(case / 'owner_plan.json') == owner_sha256, 'recovery route owner pin differs')
    item = owner.get('moving_task_capture', {}).get('inputs', {}).get('route_file', {})
    expected = str(ROOT / scope['route_path'])
    require(owner.get('route_path') == item.get('path') == manifest.get('provenance', {}).get('source_route') == expected,
            'recovery direct-route source paths differ')
    require(owner.get('route_sha256') == item.get('sha256') == manifest['provenance'].get('route_sha256')
            == sha(case / 'episode/route.json') == scope['route_sha256'],
            'recovery direct-route source hashes differ')
    require(route.get('town') == scope['town'] and route.get('scenario') == 'straight'
            and route.get('coordinate_reference') == 'base_link' and 'coordinate_alignment' not in route,
            'recovery direct-route representation differs')
    require(Path(item['path']).stem == scope['route_id'], 'recovery route identity differs')
    expected_weather = scope['plan']['fixed_arguments']['weather']
    capture_weather = manifest.get('runtime', {}).get('weather')
    catalog_weather = route.get('weather')
    require(isinstance(catalog_weather, str) and catalog_weather
            and capture_weather == expected_weather
            and owner.get('moving_task_capture', {}).get('plan', {}).get('sha256') == scope['plan_sha256'],
            'sealed owner plan or runtime capture weather differs')
    mismatch = catalog_weather != capture_weather
    return dict(route_id=scope['route_id'], source_route_path=item['path'],
        source_route_sha256=item['sha256'],
        identity_source='authenticated_owner_route_input_and_unchanged_native_catalog_artifact',
        native_coordinate_reference=route['coordinate_reference'], coordinate_alignment_synthesized=False,
        native_route_alignment_transform_performed=False,
        catalog_route_weather=catalog_weather, capture_weather=capture_weather,
        catalog_capture_weather_mismatch=mismatch,
        weather_provenance_notice=('The immutable route catalog weather is descriptive source metadata; the sealed owner plan '
            'and native runtime authenticate the actual capture weather. No route or manifest bytes were normalized.'),
        notice='Direct catalog world-space route retained byte-for-byte; per-sample map-to-base projection remains unchanged.')


def exact_replay(case, manifest, route, audit):
    """HH_260906 - Replay prospective membership twice and require exact retained native agreement."""
    states = jsonl(case / 'episode/states.jsonl')
    cameras = jsonl(case / 'episode/camera_frames.jsonl')
    plan = document(case / 'episode/moving_task_plan.json')
    replay = moving.reconstruct(plan, states, cameras, route['route'])
    tail = sum(row['capture_phase'] == 'stationary_tail' for row in states)
    require(replay.failure is None and replay.result(tail_states=tail, goal_reached=False)
            == manifest['result']['moving_task'], 'sealed moving task replay differs from native result')
    independent, projections = moving_audit.reconstruct_window(route, states, cameras)
    require(independent == audit.get('moving_scope'), 'independent moving-window replay differs from raw audit')
    require(independent['status'] == 'MOVING_SCOPE_REPLAY_CLEAR' and independent['selected_camera_span_ns'] >= 30_000_000_000,
            'recovery moving window replay is not clear')
    require([row['frame'] for row in replay.selected] == independent['selected_camera_frames'],
            'collector and independent selected memberships differ')
    require(replay.selected[0] == independent['first_selected_camera']
            and replay.selected[-1] == independent['last_selected_camera'],
            'collector and independent selected boundary anchors differ')
    phases = Counter(row['capture_phase'] for row in states)
    require(phases['stationary_warmup'] == 70 and phases['stationary_tail'] == 130
            and phases['driving'] > 0 and sum(phases.values()) == len(states),
            'native phase denominator differs from sealed capture contract')
    return states, cameras, replay.selected, projections


def target_reconstruction(states, cameras, selected):
    """HH_260906 - Rebuild every selected 6.4-second target directly from immutable 20-Hz state rows."""
    state_stamps = [round(float(row['timestamp']) * 1_000_000_000) for row in states]
    require(state_stamps == sorted(state_stamps) and len(state_stamps) == len(set(state_stamps)),
            'native state timestamps are not strictly ordered')
    by_frame = {row['frame']: row for row in cameras}
    records = []
    for anchor in selected:
        camera = by_frame[anchor['frame']]
        anchor_ns = max(round(float(value) * 1_000_000_000) for value in camera['source_timestamps'].values())
        require(anchor_ns == anchor['timestamp_ns'], 'selected anchor timestamp differs during target reconstruction')
        ego_index = bisect_right(state_stamps, anchor_ns) - 1
        require(ego_index >= 0 and state_stamps[ego_index] == anchor_ns,
                'selected anchor lacks its same-frame measured ego state')
        ego = states[ego_index]
        yaw = float(ego['yaw']); cosine, sine = math.cos(yaw), math.sin(yaw)
        positions, yaws, speeds, target_stamps = [], [], [], []
        for index in range(64):
            target = anchor_ns + (index + 1) * 100_000_000
            upper = bisect_right(state_stamps, target)
            require(upper > 0, 'selected target precedes native state support')
            left_stamp = state_stamps[upper - 1]; left = states[upper - 1]
            if left_stamp == target:
                future = {key: float(left[key]) for key in ('x', 'y', 'yaw', 'vx', 'vy')}
            else:
                require(upper < len(state_stamps), 'selected target lacks actual bracketing native future')
                right_stamp = state_stamps[upper]; right = states[upper]
                ratio = (target - left_stamp) / (right_stamp - left_stamp)
                future = {key: float(left[key]) + ratio * (float(right[key]) - float(left[key]))
                          for key in ('x', 'y', 'vx', 'vy')}
                delta = math.atan2(math.sin(float(right['yaw']) - float(left['yaw'])),
                                   math.cos(float(right['yaw']) - float(left['yaw'])))
                angle = float(left['yaw']) + ratio * delta
                future['yaw'] = math.atan2(math.sin(angle), math.cos(angle))
            dx, dy = future['x'] - float(ego['x']), future['y'] - float(ego['y'])
            positions.append([cosine * dx + sine * dy, -sine * dx + cosine * dy])
            yaws.append(math.atan2(math.sin(future['yaw'] - yaw), math.cos(future['yaw'] - yaw)))
            speeds.append(math.hypot(future['vx'], future['vy']))
            target_stamps.append(target)
        records.append(dict(frame=anchor['frame'], anchor_timestamp_ns=anchor_ns,
                            target_timestamp_ns=target_stamps, positions_base_xy_m=positions,
                            yaw_rad=yaws, speed_mps=speeds))
    return records


def verify_journals(case, audit_path, audit, cameras, selected, projections, companion_sha256, pin, decode_images):
    """HH_260906 - Authenticate all companion journals, JPEG payloads, projections, and selected 64-point targets."""
    require(set(companion_sha256) == set(COMPANIONS), 'three externally pinned audit companion SHA256 values are required')
    paths = {name: safe(audit_path.parent / name) for name in COMPANIONS}
    for name, path in paths.items():
        require(path.parent == audit_path.parent, 'audit companion escaped its authenticated directory')
        pin(path, hash_value(companion_sha256[name], name + ' SHA256'))
    image_rows = jsonl(paths['jpeg_audit.jsonl'])
    expected_paths = {relative for camera in cameras for relative in camera['images'].values()}
    require(len(image_rows) == len(expected_paths) == len(cameras) * len(contract.CAMERA_ORDER)
            and {row.get('path') for row in image_rows} == expected_paths,
            'JPEG audit denominator or referenced path set differs')
    actual_paths = {path.relative_to(case / 'episode').as_posix()
                    for path in (case / 'episode/images').rglob('*') if path.is_file()}
    require(actual_paths == expected_paths, 'native image inventory has missing or unreferenced payloads')
    if decode_images:
        from PIL import Image
    for row in image_rows:
        image_path = case / 'episode' / row['path']
        pin(image_path, hash_value(row.get('sha256'), 'JPEG payload SHA256'))
        require(row.get('referenced') is True and row.get('decoded') is True
                and row.get('expected_size') == [640, 360] and row.get('size_bytes') == image_path.stat().st_size,
                'JPEG evidence is incomplete')
        if decode_images:
            with Image.open(image_path) as opened:
                opened.load()
                require(opened.size == (640, 360) and opened.mode == row.get('mode')
                        and hashlib.sha256(opened.tobytes()).hexdigest() == row.get('decoded_pixel_sha256'),
                        'decoded native JPEG pixels differ')
    recorded_projections = jsonl(paths['native_projection.jsonl'])
    require(list(recorded_projections) == projections, 'independently replayed route projections differ')
    future_rows = jsonl(paths['future_anchor_audit.jsonl'])
    require(len(future_rows) == len(cameras)
            and [row['frame'] for row in future_rows] == [row['frame'] for row in cameras],
            'future-anchor journal denominator or order differs')
    selected_targets = target_reconstruction(states=jsonl(case / 'episode/states.jsonl'),
                                             cameras=cameras, selected=selected)
    future_by_frame = {row['frame']: row for row in future_rows}
    require(len(future_by_frame) == len(future_rows), 'duplicate frame in future-anchor journal')
    for target in selected_targets:
        row = future_by_frame[target['frame']]
        diagnostic = row.get('diagnostic', {})
        steps = diagnostic.get('steps', ())
        require(row.get('anchor_timestamp_ns') == target['anchor_timestamp_ns']
                and row.get('valid_points') == 64 and row.get('disposition') == 'full_64_point_anchor'
                and len(steps) == 64, 'selected anchor lacks an authenticated 64-point target')
        require([step.get('target_timestamp_ns') for step in steps] == target['target_timestamp_ns'],
                'selected target timestamp sequence differs')
        for index, step in enumerate(steps):
            value = step.get('metrics', {}).get('speed_limit', [None])[0]
            require(isinstance(value, (int, float)) and math.isclose(float(value), target['speed_mps'][index],
                    rel_tol=0., abs_tol=1.e-9), 'selected independently reconstructed target speed differs')
    selected_summary = moving_audit.selected_future_summary(future_rows, [row['frame'] for row in selected])
    require(selected_summary == audit.get('selected_future')
            and selected_summary['all_selected_anchors_have_actual_64_points'] is True
            and set(selected_summary['unchanged_nine_metric_failures'].values()) == {0},
            'selected future targets do not satisfy unchanged necessary checks')
    target_digest = hashlib.sha256(encoded(selected_targets)).hexdigest()
    return dict(target_sha256=target_digest, target_points=len(selected_targets) * 64,
                raw_camera_anchors=len(cameras), raw_images=len(image_rows), selected_images=len(selected) * 6)


def evidence(case_root, case_manifest_sha256, owner_plan_sha256, raw_audit, raw_audit_sha256,
             companion_sha256, *, decode_images):
    """HH_260906 - Reconstruct one complete receipt solely from externally pinned sealed evidence."""
    case, audit_path = safe(case_root), safe(raw_audit)
    external = dict(case_manifest_sha256=hash_value(case_manifest_sha256, 'case manifest SHA256'),
                    owner_plan_sha256=hash_value(owner_plan_sha256, 'owner plan SHA256'),
                    raw_audit_sha256=hash_value(raw_audit_sha256, 'raw audit SHA256'),
                    companion_sha256={name: hash_value(value, name + ' SHA256')
                                      for name, value in companion_sha256.items()})
    require(set(external['companion_sha256']) == set(COMPANIONS), 'all three companion SHA256 values are required')
    pins = {}

    def pin(path, expected=None):
        path = safe(path)
        digest = sha(path)
        require(expected is None or digest == expected, 'qualification input changed: ' + str(path))
        name = path.relative_to(ROOT).as_posix()
        require(name not in pins or pins[name] == digest, 'qualification input changed between reads: ' + name)
        pins[name] = digest
        return digest

    pin(audit_path, external['raw_audit_sha256'])
    pin(case / 'owner_plan.json', external['owner_plan_sha256'])
    pin(case / 'episode/manifest.json', external['case_manifest_sha256'])
    scope = resolve_scope(case)
    require(audit_path == case.with_name(case.name + '_analysis') / 'summary.json',
            'raw audit path differs from the sealed case-specific analysis path')
    audit = document(audit_path)
    require(audit.get('schema') == 'carla.moving_capture_diagnostic.v1'
            and audit.get('status') == 'DIAGNOSED_NOT_ADMITTED'
            and audit.get('moving_scope_necessary_checks_clear') is True,
            'raw moving necessary checks are not clear')
    require(audit.get('trial') == scope['case'] and audit.get('owner_plan_sha256') == external['owner_plan_sha256'],
            'raw audit is bound to a different case or owner')
    sealed = audit.get('sealed_capture_scope', {})
    require(sealed == {key: scope[key] for key in ('slot_id', 'town', 'route_id', 'route_sha256')}
            | {'plan_schema': moving.plan_contract(scope['slot_id'])['schema']},
            'raw audit sealed-slot scope differs')
    pin_source_manifest(case, audit, pin)
    manifest = document(case / 'episode/manifest.json')
    scope_manifest(manifest, scope)
    owner = document(case / 'owner_plan.json')
    owner_result = document(case / 'owner_result.json')
    require(owner.get('capture_mode') == 'expert' and owner.get('quality') == 'Epic'
            and owner.get('map') == scope['town'] and owner.get('route_sha256') == scope['route_sha256'],
            'owned recovery capture scope differs')
    require(owner_result.get('exit_code') == 0
            and owner_result.get('moving_task_capture', {}).get('capture_complete_pending_qa') is True
            and owner_result['moving_task_capture'].get('dataset_admission_granted') is False
            and owner_result['moving_task_capture'].get('observed_dataset_conversion_performed') is False,
            'owned recovery capture did not complete as pending QA')
    pin(case / 'episode/manifest.json', owner_result['moving_task_capture']['final_manifest_sha256'])
    for name, expected in owner['source_sha256'].items():
        pin(case / 'provenance' / name, expected)
        pin(ROOT / name, expected)
    refs = owner['moving_task_capture']
    require(set(refs['inputs']) == {'route_file', 'mapping', 'calibration'}, 'owned recovery input closure differs')
    for item in [refs['plan'], *refs['inputs'].values()]:
        source = safe(item['path'])
        require(source.is_relative_to(ROOT), 'owned recovery source escapes repository')
        pin(source, item['sha256'])
        pin(case / item['archived_path'], item['sha256'])
    require(isinstance(audit.get('diagnostic_source', {}).get('files'), dict)
            and isinstance(audit.get('diagnostic_source', {}).get('functions'), dict),
            'raw audit diagnostic provenance is absent')
    route = document(case / 'episode/route.json')
    route_identity = direct_route_identity(case, manifest, route, owner, scope, external['owner_plan_sha256'])
    states, cameras, selected, projections = exact_replay(case, manifest, route, audit)
    journal_facts = verify_journals(case, audit_path, audit, cameras, selected, projections,
                                    external['companion_sha256'], pin, decode_images)
    source_pins = {name: sha(ROOT / name) for name in SOURCE_PATHS}
    for name, expected in (pins | source_pins).items():
        require(sha(ROOT / name) == expected, 'qualification source/input changed during inspection: ' + name)
    return dict(schema=SCHEMA, status='QUALIFIED_FOR_COMMON10_CONVERSION', slot_id=scope['slot_id'],
        case=scope['case'], plan_sha256=scope['plan_sha256'], owner_plan_sha256=external['owner_plan_sha256'],
        native_manifest_sha256=external['case_manifest_sha256'], raw_audit_path=audit_path.relative_to(ROOT).as_posix(),
        raw_audit_sha256=external['raw_audit_sha256'], companion_sha256=external['companion_sha256'],
        source_sha256=source_pins, input_sha256=pins, route_identity=route_identity,
        selected_camera_frames=[row['frame'] for row in selected], selected_anchors=selected,
        selected_camera_anchors=len(selected), selected_span_ns=selected[-1]['timestamp_ns'] - selected[0]['timestamp_ns'],
        context_only_camera_anchors=len(cameras) - len(selected), **journal_facts,
        full_raw_failures_retained=audit['full_raw_future_all_original_phases'],
        conversion_only=True, prepared_dataset_validation_performed=False, training_run_performed=False,
        catalog_goal_reached=False, model_or_vehicle_release_approved=False,
        notice='This receipt authorizes one sealed TRAIN-only staged conversion; it is not model, vehicle, validation, or TEST approval.')


def recheck(qualification):
    """HH_260906 - Rehash the immutable receipt closure immediately before and after conversion work."""
    require(sha(qualification['receipt_path']) == qualification['receipt_sha256'], 'recovery qualification receipt changed')
    document_value = qualification['document']
    for name, expected in (document_value['input_sha256'] | document_value['source_sha256']).items():
        require(sha(ROOT / name) == expected, 'recovery qualification source/input changed: ' + name)


def authorize(args):
    """HH_260906 - Authorize exactly one TRAIN episode and one slot-specific copy/planning output."""
    path = getattr(args, 'moving_qualification', None)
    expected = getattr(args, 'moving_qualification_sha256', None)
    require(path is not None and expected is not None, 'paired moving qualification path and external SHA required')
    path = safe(path)
    require(sha(path) == hash_value(expected, 'qualification receipt SHA256'),
            'recovery qualification receipt external SHA differs')
    receipt = document(path)
    require(receipt.get('schema') == SCHEMA, 'stale or foreign recovery qualification receipt schema')
    slot_id = receipt.get('slot_id')
    require(slot_id in QUALIFIABLE_SLOT_IDS, 'receipt is not for one successful sealed recovery slot')
    case = slot_case(slot_id)
    require(len(args.episode) == 1 and args.episode[0].split == 'train'
            and safe(args.episode[0].path) == case / 'episode',
            'recovery conversion permits one exact TRAIN input only')
    require(args.image_mode == 'copy' and args.validate_mode == 'planning',
            'recovery conversion requires copy/planning mode')
    output = safe(args.output)
    require(output == prepared_path(slot_id), 'recovery conversion requires its fixed slot-specific output')
    audit_path = safe(ROOT / receipt['raw_audit_path'])
    for protected in (case, audit_path.parent, path):
        require(not output.is_relative_to(protected) and not protected.is_relative_to(output),
                'recovery output overlaps protected evidence')
    expected_receipt = evidence(case, receipt['native_manifest_sha256'], receipt['owner_plan_sha256'],
        audit_path, receipt['raw_audit_sha256'], receipt['companion_sha256'], decode_images=False)
    require(receipt == expected_receipt, 'recovery receipt differs from independently reconstructed evidence')
    result = dict(receipt_path=str(path), receipt_sha256=expected, document=receipt)
    recheck(result)
    return result


def verify_native_scope(spec, manifest, qualification):
    """HH_260906 - Rebind the converter's loaded native episode to the authenticated recovery receipt."""
    receipt = qualification['document']; slot_id = receipt.get('slot_id')
    scope = resolve_scope(slot_case(slot_id))
    scope_manifest(manifest, scope)
    require(spec.split == 'train' and safe(spec.path) == slot_case(slot_id) / 'episode',
            'wrong recovery native input')
    require(receipt['case'] == scope['case'] and receipt['plan_sha256'] == scope['plan_sha256']
            and receipt['native_manifest_sha256'] == sha(spec.path / 'manifest.json')
            and receipt['conversion_only'] is True, 'recovery receipt scope differs')


def native_route_id(spec, manifest, route, qualification):
    """HH_260906 - Return only the owner-authenticated unchanged catalog route identity."""
    verify_native_scope(spec, manifest, qualification)
    receipt = qualification['document']; case = spec.path.parent
    scope = resolve_scope(case)
    identity = direct_route_identity(case, manifest, route, document(case / 'owner_plan.json'),
                                     scope, receipt['owner_plan_sha256'])
    require(identity == receipt['route_identity'], 'recovery route identity differs from receipt')
    return identity['route_id']


def verify_weather_provenance(spec, manifest, route, qualification):
    """HH_260906 - Authorize catalog/capture weather provenance only for one fully authenticated recovery receipt."""
    verify_native_scope(spec, manifest, qualification)
    receipt = qualification['document']; case = spec.path.parent
    scope = resolve_scope(case)
    identity = direct_route_identity(case, manifest, route, document(case / 'owner_plan.json'),
                                     scope, receipt['owner_plan_sha256'])
    require(identity == receipt['route_identity'], 'recovery weather provenance differs from receipt')
    return dict(catalog_route_weather=identity['catalog_route_weather'],
                capture_weather=identity['capture_weather'],
                catalog_capture_weather_mismatch=identity['catalog_capture_weather_mismatch'],
                weather_provenance_notice=identity['weather_provenance_notice'])


def independently_expected(native, sample, state_by_stamp, stamps, polyline, route_hash, contract_data):
    """HH_260906 - Reconstruct one prepared ego, navigation record, and all 64 planning points independently."""
    anchor = sample['anchor_timestamp_ns']; ego_stamp = stamps[bisect_right(stamps, anchor) - 1]
    ego = state_by_stamp[ego_stamp]; yaw = ego['yaw']; cosine, sine = math.cos(yaw), math.sin(yaw)
    positions, yaws, speeds, times = [], [], [], []
    for index in range(64):
        stamp = anchor + (index + 1) * 100_000_000; upper = bisect_right(stamps, stamp)
        require(upper > 0, 'prepared target precedes native support')
        left_stamp = stamps[upper - 1]; left = state_by_stamp[left_stamp]
        if left_stamp == stamp:
            future = {key: float(left[key]) for key in ('x', 'y', 'yaw', 'vx', 'vy')}
        else:
            require(upper < len(stamps), 'prepared target lacks actual raw future')
            right_stamp = stamps[upper]; right = state_by_stamp[right_stamp]
            ratio = (stamp - left_stamp) / (right_stamp - left_stamp)
            future = {key: float(left[key]) + ratio * (float(right[key]) - float(left[key]))
                      for key in ('x', 'y', 'vx', 'vy')}
            delta = math.atan2(math.sin(float(right['yaw']) - float(left['yaw'])),
                               math.cos(float(right['yaw']) - float(left['yaw'])))
            angle = float(left['yaw']) + ratio * delta
            future['yaw'] = math.atan2(math.sin(angle), math.cos(angle))
        dx, dy = future['x'] - float(ego['x']), future['y'] - float(ego['y'])
        positions.append([cosine * dx + sine * dy, -sine * dx + cosine * dy])
        yaws.append(math.atan2(math.sin(future['yaw'] - yaw), math.cos(future['yaw'] - yaw)))
        speeds.append(math.hypot(future['vx'], future['vy'])); times.append(stamp)
    quaternion = [0., 0., math.sin(.5 * yaw), math.cos(.5 * yaw)]
    route, goal, arc = contract._canonical_route_in_base(tuple(tuple(point) for point in polyline),
        position_m=(ego['x'], ego['y'], ego['z']), orientation_xyzw=quaternion,
        contract=contract_data, context='independent recovery reconstruction')
    expected_ego = dict(timestamp_ns=ego_stamp, frame_id='map', child_frame_id='base_link',
        position_m=[ego['x'], ego['y'], ego['z']], orientation_xyzw=quaternion,
        linear_velocity_base_mps=[ego['vx'], ego['vy'], 0.], angular_velocity_base_radps=[0., 0., ego['yaw_rate']],
        linear_acceleration_base_mps2=[ego['ax'], ego['ay'], 0.], steering_tire_angle_rad=ego['steering_tire_angle_rad'])
    expected_navigation = dict(route_id=native.route_id, route_source='episode_route_geometry',
        route_source_timestamp_ns=stamps[0], route_geometry_sha256=route_hash, route_anchor_arc_m=arc,
        command=ego['command'], command_timestamp_ns=ego_stamp, speed_limit_mps=ego['speed_limit_mps'],
        route_polyline_base_m=[list(point) for point in route], goal_base_m=list(goal))
    expected_planning = dict(available=True, dt_s=.1, positions_base_xy_m=positions, yaw_rad=yaws,
        speed_mps=speeds, valid=[True] * 64, target_timestamp_ns=times, invalid_reason=[None] * 64)
    require(sample['ego'] == expected_ego and sample['navigation'] == expected_navigation
            and sample['labels']['planning'] == expected_planning,
            'native-to-prepared ego/navigation/64-target reconstruction differs')


def validate_prepared(prepared, native, qualification, native_report):
    """HH_260906 - Validate every prepared sample, copied JPEG, and target before a staged result may remain."""
    from scripts.e2e import audit_portable_target_feasibility as targets
    recheck(qualification); prepared = safe(prepared); receipt = qualification['document']
    expected_count = receipt['selected_camera_anchors']
    loaded = dataset.load_training_examples(prepared, split='train', mode='planning', check_image_hashes=True)
    require(len(loaded.examples) == expected_count and native_report['sample_count'] == expected_count
            and native_report['episode_count'] == 1, 'unchanged validator/loader TRAIN denominator differs')
    dataset_doc = document(prepared / 'dataset.json')
    require(len(dataset_doc['episodes']) == 1, 'prepared recovery dataset must contain one episode')
    episode_path = prepared / dataset_doc['episodes'][0]['manifest']; episode_root = episode_path.parent
    episode = document(episode_path); samples = jsonl(episode_root / 'samples.jsonl')
    collection = document(episode_root / 'collection_config.json')
    require(episode['split'] == 'train'
            and episode['source_provenance']['adapter_id'] == 'carla_sealed_moving_recovery_to_common10_v1'
            and collection.get('adapter_id') == 'carla_sealed_moving_recovery_to_common10_v1',
            'prepared recovery split or adapter identity differs')
    require(collection['native_capture_contract'] == native.manifest['capture_contract']
            and collection['native_runtime'] == native.manifest['runtime']
            and collection['native_result'] == native.manifest['result'],
            'prepared native capture/runtime/result metadata changed')
    require(sha(episode_root / 'native_manifest.json') == native.source_hashes['manifest.json']
            and sha(episode_root / 'native_route.json') == native.source_hashes['route.json']
            and sha(episode_root / 'moving_qualification.json') == qualification['receipt_sha256'],
            'prepared raw provenance copy differs')
    timeline = adapter._state_timeline(native); state_by_stamp = dict(timeline); stamps = list(state_by_stamp)
    polyline = []
    for point in native.route['route']:
        xy = [float(point['x']), float(point['y'])]
        if not polyline or math.dist(polyline[-1], xy) > 1.e-9:
            polyline.append(xy)
    require(document(episode_root / 'route_geometry.json')
            == dict(route_id=native.route_id, frame_id='map', polyline_m=polyline),
            'prepared route geometry differs from native route')
    raw_manifest = document(episode_root / 'source_manifest.json')
    require(len(raw_manifest['camera_frames']) == receipt['raw_images'], 'full raw JPEG provenance denominator differs')
    case = receipt['case']
    for image in raw_manifest['camera_frames']:
        expected = receipt['input_sha256'][case + '/episode/' + image['source_uri']]
        require(image['source_payload_sha256'] == expected, 'raw context JPEG digest changed')
    by_frame = {row['frame']: row for row in native.camera_frames}; copied = []; audits = []
    require(len(samples) == len(receipt['selected_anchors']) == expected_count, 'selected sample denominator differs')
    contract_data = contract.load_contract()
    for index, (sample, selected) in enumerate(zip(samples, receipt['selected_anchors'])):
        require(sample['sequence_index'] == index and sample['anchor_timestamp_ns'] == selected['timestamp_ns'],
                'prepared selected ordering or timestamp differs')
        camera = by_frame[selected['frame']]
        independently_expected(native, sample, state_by_stamp, stamps, polyline,
                               episode['route_geometry_sha256'], contract_data)
        for item, name in zip(sample['camera_bundle'], contract.CAMERA_ORDER):
            source = native.spec.path / camera['images'][name]; destination = episode_root / item['path']
            require(item['name'] == name and item['source_frame_id'] == f"carla:{selected['frame']:010d}:{name}"
                    and sha(source) == sha(destination) == item['sha256'], 'prepared JPEG identity differs')
            require(source.stat().st_ino != destination.stat().st_ino and destination.stat().st_nlink == 1,
                    'prepared recovery JPEGs must be independent copies')
            copied.append(destination)
        audited = targets.audit_sample(sample, route_length_m=native.route['route_length_m'],
            episode_start_ns=episode['start_timestamp_ns'], episode_end_ns=episode['end_timestamp_ns'],
            tail_start_ns=next(stamp for stamp, state in timeline if state['capture_phase'] == 'stationary_tail'))
        audited['episode_id'] = native.episode_id; audits.append(audited)
    require(len(copied) == receipt['selected_images']
            and set(copied) == {path for path in (episode_root / 'images').rglob('*') if path.is_file()},
            'prepared selected JPEG denominator differs')
    accounting = episode['capture_accounting']
    require(accounting['raw_anchor_count'] == receipt['raw_camera_anchors']
            and accounting['selected_bundle_count'] == expected_count
            and accounting['ineligible_anchor_reasons']
            == {'moving_task_fixed_window_context_only': receipt['context_only_camera_anchors']},
            'prepared context-only ledger differs')
    measured = targets.summarize_samples(audits, 64)
    require(measured['valid_step_count'] == receipt['target_points'] and measured['invalid_step_count'] == 0
            and all(value['violating_step_count'] == 0 for value in measured['metrics'].values()),
            'prepared recovery targets fail unchanged necessary bounds')
    recheck(qualification)
    return dict(schema='carla.moving_recovery_common10_conversion_result.v1',
        status='QUALIFIED_COMMON10_PLANNING_CONVERSION', qualified_for_common10_planning_conversion=True,
        slot_id=receipt['slot_id'], qualification_receipt_sha256=qualification['receipt_sha256'],
        dataset_manifest_sha256=sha(prepared / 'dataset.json'),
        validation_report_sha256=sha(prepared / 'validation_report.json'),
        train_loader_fingerprint_sha256=loaded.fingerprint_sha256, train_examples=expected_count,
        val_examples=0, test_examples=0, exact_native_ego_navigation_and_target_reconstruction=True,
        rederived_target_points=receipt['target_points'], raw_camera_anchors=receipt['raw_camera_anchors'],
        selected_camera_anchors=expected_count, context_only_camera_anchors=receipt['context_only_camera_anchors'],
        raw_image_hashes_preserved=receipt['raw_images'], independently_copied_selected_images=receipt['selected_images'],
        unchanged_target_audit=measured, full_raw_failures_retained=receipt['full_raw_failures_retained'],
        native_goal_reached=False, native_pending_metadata_unchanged=True, training_run_performed=False,
        route_identity=receipt['route_identity'], native_route_bytes_unchanged=True,
        model_or_vehicle_release_approved=False, manual_release_review_required=True,
        notice='One sealed recovery TRAIN capture only; no validation, TEST, learned-control, or real-vehicle approval.')


def main(argv=None):
    """HH_260906 - Create one fresh receipt after full read-only source and pixel authentication."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--case-root', type=Path, required=True)
    parser.add_argument('--case-manifest-sha256', required=True)
    parser.add_argument('--owner-plan-sha256', required=True)
    parser.add_argument('--raw-audit', type=Path, required=True)
    parser.add_argument('--raw-audit-sha256', required=True)
    parser.add_argument('--jpeg-audit-sha256', required=True)
    parser.add_argument('--future-anchor-audit-sha256', required=True)
    parser.add_argument('--native-projection-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    output = safe(args.output)
    require(not output.exists(), 'recovery qualification receipt must be fresh')
    for protected in (safe(args.case_root), safe(args.raw_audit).parent):
        require(not output.is_relative_to(protected) and not protected.is_relative_to(output),
                'recovery receipt overlaps original evidence')
    companions = {'jpeg_audit.jsonl': args.jpeg_audit_sha256,
                  'future_anchor_audit.jsonl': args.future_anchor_audit_sha256,
                  'native_projection.jsonl': args.native_projection_sha256}
    value = evidence(args.case_root, args.case_manifest_sha256, args.owner_plan_sha256,
                     args.raw_audit, args.raw_audit_sha256, companions, decode_images=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(dict(receipt=str(output), sha256=sha(output), status=value['status'],
                          slot_id=value['slot_id'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
