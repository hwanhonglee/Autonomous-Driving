#!/usr/bin/env python3
"""HH_260906 - Plan map coverage without launching CARLA, reading TEST payloads, or authorizing training."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import inventory_carla_training_maps as inventory

PROTECTED_MAPS = frozenset({'town04', 'town15'})
ACTIVE_TRAIN_MAPS = frozenset({'town01', 'town07', 'c_track_1_0_7'})
ACTIVE_DEV_MAPS = frozenset({'town03'})
SCENARIOS = ('straight', 'left', 'right', 'stop_hold', 'restart')


def require(value, message):
    if not value:
        raise ValueError(message)


def checked_path(path):
    path = Path(path).absolute()
    require('..' not in path.parts, 'parent traversal is forbidden')
    require(not any(part.is_symlink() for part in (path, *path.parents)), 'symlink paths are forbidden')
    return path


def strict_json(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    def invalid(value):
        raise ValueError('non-finite JSON number: ' + value)
    document = json.loads(payload, object_pairs_hook=pairs, parse_constant=invalid)
    def finite(value):
        if isinstance(value, float):
            require(math.isfinite(value), 'non-finite JSON number')
        elif isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, list):
            for item in value:
                finite(item)
    finite(document)
    return document


def catalog_counts(catalog_root, map_id, canonical_name, ledger):
    # HH_260906 - Refuse protected maps BEFORE constructing or opening any catalog path.
    require(map_id not in PROTECTED_MAPS, 'protected map catalog access is forbidden')
    require(map_id in inventory.MAP_CONTRACT, 'unknown map identifier')
    counts = Counter({name: 0 for name in ('straight', 'left', 'right')})
    identities = set()
    for kind in ('straight', 'turn'):
        path = checked_path(Path(catalog_root) / map_id / 'catalog' / kind / 'route_catalog.json')
        require(path.is_file(), 'missing catalog: ' + str(path))
        require(path.stat().st_size <= 16 * 1024 * 1024, 'oversized catalog')
        payload = path.read_bytes()
        document = strict_json(payload)
        require(document.get('status') == 'complete' and document.get('map_id') == map_id
                and document.get('canonical_name') == canonical_name, 'catalog identity/status mismatch')
        routes = document.get('routes')
        require(isinstance(routes, list), 'catalog routes must be a list')
        for route in routes:
            require(isinstance(route, dict), 'catalog route must be an object')
            require(route.get('status') == 'ready', 'catalog contains a non-ready route')
            scenario, identity = route.get('scenario'), route.get('id')
            require(scenario in (('straight',) if kind == 'straight' else ('left', 'right')), 'catalog scenario mismatch')
            require(isinstance(identity, str) and identity and identity not in identities, 'duplicate/invalid route identity')
            identities.add(identity)
            counts[scenario] += 1
        # HH_260906 - Route counts are metadata only, not new route-payload validation or independent drives.
        ledger[str(path.relative_to(catalog_root))] = hashlib.sha256(payload).hexdigest()
    return dict(counts)


def build_plan(manifest, local_assets, catalog_root):
    entries = manifest['maps']
    require(len({row['id'] for row in entries}) == len(entries), 'duplicate manifest map')
    require({row['id'] for row in entries} == set(inventory.MAP_CONTRACT), 'canonical map set differs')
    assets = {row['id']: row for row in local_assets['maps']}
    require(len(assets) == len(local_assets['maps']) and set(assets) == set(inventory.MAP_CONTRACT), 'asset inventory map set differs')
    profiles = {row['id']: row for row in local_assets['profiles']}
    require(len(profiles) == len(local_assets['profiles']) and set(profiles) == set(inventory.PROFILE_CONTRACT), 'runtime profile set differs')
    maps, ledger = [], {}
    for entry in entries:
        name = entry['id']
        asset = assets[name]
        require(asset['assets_complete'] is None or type(asset['assets_complete']) is bool, 'asset completeness must be boolean/null')
        protected = name in PROTECTED_MAPS or entry['split'] == 'test'
        role = ('LOCKED_HOLDOUT' if protected else 'ACTIVE_TRAIN_MAP' if name in ACTIVE_TRAIN_MAPS
                else 'ACTIVE_DEVELOPMENT_MAP' if name in ACTIVE_DEV_MAPS else 'UNASSIGNED_NEW_COHORT')
        reasons = []
        if protected:
            reasons.append('LOCKED_HOLDOUT_NO_COLLECTION_OR_PAYLOAD_READ')
        if entry['status'] == 'unavailable' or entry['split'] == 'excluded':
            reasons.append('UNAVAILABLE_OR_EXCLUDED')
        if asset['assets_complete'] is not True:
            reasons.append('REQUIRED_MAP_ASSETS_NOT_CONFIRMED')
        if entry['server_profile'] != 'packaged_0915':
            reasons.append('OUTSIDE_PACKAGED_CAPTURE_PROFILE')
        else:
            profile = profiles['packaged_0915']
            if not all(profile.get(field) is True for field in ('executable_exists', 'executable_is_runnable',
                    'python_api_exists', 'python_api_sha256_matches')):
                reasons.append('PACKAGED_EXECUTABLE_OR_API_NOT_CONFIRMED')
        counts = None
        if not reasons:
            counts = catalog_counts(catalog_root, name, entry['canonical_name'], ledger)
        conflict = ((name in ACTIVE_DEV_MAPS and entry['split'] != 'validation')
                    or (protected and entry['split'] != 'test'))
        maps.append(dict(map_id=name, canonical_name=entry['canonical_name'], declared_status=entry['status'],
            assets_complete=asset['assets_complete'], manifest_split=entry['split'], active_role=role,
            split_policy_conflict=conflict, historical_catalog_counts=counts,
            catalog_metadata_inspected=counts is not None, route_payloads_inspected=False, learned_closed_loop_passes=0,
            cells=[dict(scenario=scenario, execution_allowed=False, training_data_approved=False,
                status='BLOCKED', blockers=reasons + (
                    ['NO_CATALOG_ROUTE_FOR_DIRECTION'] if counts is not None and scenario in counts and counts[scenario] == 0 else []
                ) + (['STOP_RESTART_INTENT_PROTOCOL_NOT_APPROVED'] if scenario in ('stop_hold', 'restart') else [])
                + ['FRESH_EPISODE_SPLIT_AND_CAPTURE_PLAN_REQUIRED', 'RAW_QUALITY_ADMISSION_NOT_COMPLETE'])
                for scenario in SCENARIOS]))
    return dict(schema='portable_e2e.map_coverage_plan.v1', status='PLAN_ONLY_NOT_EXECUTABLE',
        generated_at_utc=datetime.now(timezone.utc).isoformat(),
        map_count=len(maps), scenario_cell_count=len(maps)*len(SCENARIOS), maps=maps,
        constraints=dict(camera_hz=10, physics_hz=20, nominal_speed_ceiling_kph=30,
            curve_speed_policy='reduce_for_geometry_not_constant_30kph', quality='Epic',
            initial_weather='ClearNoon', training_physical_gpu=0, personal_venv_only=True,
            installations_allowed=False, source_raw_and_failed_runs_preserved=True,
            owned_local_simulator_only=True, learned_control_allowed=False,
            automatic_model_promotion=False, test_payload_access_allowed=False,
            real_vehicle_control_allowed=False, speed_60kph_deferred=True,
            imagery='actual_ego_centered_route_and_trail; label_BasicAgent_VAD_shadow_or_learned'),
        priority=['town07/straight', 'c_track_1_0_7/left', 'town03/right:development_only'],
        source_catalog_sha256=ledger, world_rpc_performed=False, execution_commands=[],
        training_data_approved=False, all_maps_validated=False,
        limitations=['This new planner does not alter historical runners or the legacy manifest.',
            'Zero learned passes means none admitted by this plan; this tool does not search or evaluate model-driving evidence.',
            'Map files and route catalogs do not establish current renderability or learned driving.',
            'No automatic split migration: Town03 remains development and Town04 remains locked.',
            'A stop/restart intent protocol and independent episode groups remain prerequisites.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=inventory.DEFAULT_MANIFEST)
    parser.add_argument('--catalog-maps-root', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    args = parser.parse_args(argv)
    output, manifest_path = checked_path(args.output_json), checked_path(args.manifest)
    catalogs = checked_path(args.catalog_maps_root)
    require(not output.exists(), 'output must be fresh')
    require(output != manifest_path and not output.is_relative_to(catalogs), 'output overlaps evidence')
    # HH_260906 - Preserve current filesystem observations separately from the manifest's historical assertions.
    source_files = (manifest_path, Path(__file__).absolute(), Path(inventory.__file__).absolute())
    source_pins = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_files}
    manifest, _ = inventory.load_manifest(manifest_path)
    local_assets = inventory.collect_static_inventory(manifest)
    report = build_plan(manifest, local_assets, catalogs)
    for path, expected in source_pins.items():
        require(hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, 'source changed during planning')
    for name, expected in report['source_catalog_sha256'].items():
        require(hashlib.sha256(checked_path(catalogs / name).read_bytes()).hexdigest() == expected, 'catalog changed during planning')
    report['source_sha256_before'] = source_pins
    report['source_sha256_after'] = dict(source_pins)
    report['local_filesystem_snapshot'] = local_assets
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(status=report['status'], map_count=report['map_count'], cells=report['scenario_cell_count'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
