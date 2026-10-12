#!/usr/bin/env python3
"""HH_261005 - Audit exact candidate exposure using a sealed metadata-only list."""
import argparse
import hashlib
import json
import os
import re
import stat
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / 'storage/experiments/plans/2026-10-05/rte01_good6000_exposure_metadata_v2'
ALLOWLIST = PLAN / 'named_metadata_allowlist.json'
AUTHORIZATION = PLAN / 'authorization.json'
OUTPUT = PLAN / 'exposure_ledger.json'
SCHEMA = 'portable_e2e.rte01_good6000_exposure_metadata.20261005.v2'
GUARD = 'learning/config/site_split_guard_20260929_v2.json'
GUARD_SHA = '323b68c690724bdb63a9955c9573a249d5c49f5001160a421e797249dce7ec0b'
MAX_FILE_BYTES = 16 * 1024 * 1024
MAX_TOTAL_BYTES = 256 * 1024 * 1024
MAX_FILES = 1200
IDENTITIES = {
    'GOOD6000_weights': 'b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625',
    'GOOD6000_checkpoint': '43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88',
    'recovery3000_weights': '0bdd747dffa6e37196e0ee6afb25fa9930d6aba6ca6f787c779ccfbeae3d4dc9',
    'recovery3000_checkpoint': 'a17544d99fd87467e4afe1c051dd598f72ab9cd0d79794f34d10291e1edce52f',
    'balanced3000_weights': '4e1623e5156b2a19b546f1889d7a70e9c7b312f72fbf7a0e5a99ccda7d8e5735',
    'balanced3000_checkpoint': 'c87c582fc4823d8ef3a00faf103c7aaa18481f2e1bfb7d38fea2f4d7136a99e9',
    'random200_checkpoint': '6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece',
}
REVERSE_IDENTITIES = {value: key for key, value in IDENTITIES.items()}
RESERVED_SITES = ('Town04', 'Town10HD_Opt', 'Town15')
META_NAMES = {'declaration.json', 'manifest.json', 'learned_summary.json',
              'summary.json', 'metadata.json', 'owner_started.json',
              'owner_plan.json', 'declared_plan.json', 'authorization.json',
              'started.json', 'evaluation_opened_after_final.json'}
EVALUATION_METADATA_NAMES = {
    'known_dev_raw_before.json', 'known_dev_raw_after.json', 'known_dev_raw_final.json',
    'known_moving_dev599_raw_before.json', 'known_moving_dev599_raw_final.json',
    'known599_before.json', 'known599_final.json', 'legacy_before.json', 'legacy_final.json',
    'v4_before.json', 'v4_final.json', 'recovery_train_before.json', 'recovery_train_final.json',
    'recovery_holdout_before.json', 'recovery_holdout_final.json',
    'legacy_train_raw_before.json', 'legacy_train_raw_final.json',
    'v4_train_joint_before.json', 'v4_train_joint_final.json', 'endpoint_diagnostic_screens.json',
}
EXTRA_METADATA = {
    GUARD, *(f'docs/assets/validation/2026-10-0{day}/README.md' for day in (1, 2, 4)),
    *(f'storage/datasets/prepared/{name}/dataset.json' for name in (
        'town07-low-speed-cruise-v4-research-train-20261001-v1',
        'town07-signed-recovery-train-20261001-v4',
        'town07-signed-recovery-holdout-20261001-v4')),
    'storage/experiments/plans/2026-10-05/rte01_test_only_metadata_qualification_v1/metadata_qualification.json',
}
ANCESTRAL_RUNS = (
    'storage/experiments/training/2026-10-01/temporal_v2_raw_bootstrap_v1/',
    'storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_v1/',
    'storage/experiments/training/2026-10-01/temporal_route_recovery_v3_v1/',
    'storage/experiments/training/2026-10-02/temporal_route_joint_v1/',
)
# HH_261005 - Exclude numeric diagnostic/payload subtrees before extracting any value.
BLOCKED_WORDS = ('loss', 'prediction', 'label', 'metrics', 'true_future', 'ground_truth',
                 'raw_output', 'numeric_output', 'tensor_values', 'curvature_values')
BLOCKED_KEYS = {'states', 'samples', 'frames', 'images', 'pixels', 'prefill_records',
                'raw', 'camera_frames', 'learned_commands', 'native_states', 'timings'}
ACTIVE_LEAVES = {'source_checkpoint_sha256', 'checkpoint_sha256', 'npz_sha256',
                 'model_weights_sha256', 'weights_sha256'}
SELECTION_KEYS = {'checkpoint_selection', 'selection_rule', 'endpoint_selection',
                  'checkpoint_selection_by_holdout', 'checkpoint_selection_by_dev',
                  'holdout_selected_checkpoint', 'holdout_fitted',
                  'holdout_training_or_checkpoint_selection_allowed'}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def require_sha(value):
    if not isinstance(value, str) or re.fullmatch('[0-9a-f]{64}', value) is None:
        raise ValueError('expected exact lowercase SHA256')
    return value


def metadata_path_allowed(relative):
    if not isinstance(relative, str):
        return False
    path = Path(relative)
    if path.is_absolute() or '..' in path.parts or str(path) != relative:
        return False
    if relative in EXTRA_METADATA:
        return True
    if path.stem in {'states', 'camera_frames', 'learned_commands', 'samples', 'labels',
                     'future_labels', 'raw_predictions', 'model_state', 'checkpoint'}:
        return False
    if any(part in path.parts for part in ('source', 'initial_lineage', 'provenance', 'images')):
        return False
    if any(part.startswith('derived_') for part in path.parts) or path.name == 'moving_task_plan.json':
        return False
    if re.match(r'^storage/experiments/(plans|validation|training|diagnostics)/2026-10-0[1-5]/', relative) is None:
        return False
    return (path.name in META_NAMES or path.name.endswith('_plan.json') or
            ('/training/' in relative and path.name in EVALUATION_METADATA_NAMES) or
            ('/plans/' in relative and path.suffix == '.json'))


def read_regular(path, limit=MAX_FILE_BYTES):
    path = Path(path)
    if any(candidate.is_symlink() for candidate in (path, *path.parents)):
        raise ValueError('symlink metadata path')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
            raise ValueError('metadata type/size exceeds fixed bound')
        data = bytearray()
        while len(data) <= limit:
            block = os.read(fd, min(1024 * 1024, limit + 1 - len(data)))
            if not block:
                break
            data.extend(block)
        after = os.fstat(fd)
        fields = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns')
        current = path.lstat()
        if len(data) != before.st_size or any(getattr(before, f) != getattr(after, f) or
                getattr(after, f) != getattr(current, f) for f in fields):
            raise ValueError('metadata generation changed during read')
        return bytes(data)
    finally:
        os.close(fd)


def parse_json(data):
    def pairs(items):
        output = {}
        for key, value in items:
            if key in output:
                raise ValueError('duplicate metadata key')
            output[key] = value
        return output
    def nonfinite(_):
        raise ValueError('nonfinite JSON metadata')
    return json.loads(data, object_pairs_hook=pairs, parse_constant=nonfinite)


def projected_strings(value, path=()):
    if isinstance(value, dict):
        for key, child in value.items():
            if key.lower() not in BLOCKED_KEYS and not any(word in key.lower() for word in BLOCKED_WORDS):
                yield from projected_strings(child, path + (key,))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from projected_strings(child, path + (str(index),))
    elif isinstance(value, str):
        yield path, value
    # Numeric diagnostic values are deliberately never yielded or reported.


def identity_class(path):
    lowered = '/'.join(path).lower()
    if any(word in lowered for word in ('asset_sha256', 'source_sha256', 'source_hashes',
            'immutable_input_sha256', 'all32_pins', 'all_pins')) or '/source/' in lowered:
        return 'source_archive_reference_not_active_model'
    if 'parent' in lowered or 'ancestry' in lowered:
        return 'ancestry_or_descendant_parent_reference'
    if path and (path[-1] in ACTIVE_LEAVES or
            path[:1] in (('new_fixed_endpoint_export_sha256',), ('selected_four_endpoint_sha256',))):
        return 'direct_model_binding'
    return 'other_context_reference_not_proof_of_inference'


def extract_record(relative, doc):
    if not isinstance(doc, dict):
        return {'path': relative, 'nonobject_metadata': True, 'matches': [], 'site': None}
    matches = []
    site_identifiers = set()
    for path, value in projected_strings(doc):
        if value in REVERSE_IDENTITIES:
            matches.append({'field': list(path), 'identity': REVERSE_IDENTITIES[value],
                            'kind': identity_class(path)})
        if path and path[-1] in ('episode_id', 'dataset_path', 'source_route', 'route_file', 'path'):
            site_identifiers.update(re.findall(r'(?i)(?:carla_|/|^)(town\d+(?:hd_opt)?|c[_-]track)(?:_|/|$)', value))
    runtime = doc.get('runtime')
    site = doc.get('town') or (runtime.get('town') if isinstance(runtime, dict) else None)
    if not isinstance(site, str):
        site = None
    selections = {}
    def select(value, path=()):
        if isinstance(value, dict):
            for key, child in value.items():
                if key.lower() in BLOCKED_KEYS or any(word in key.lower() for word in BLOCKED_WORDS):
                    continue
                if key in SELECTION_KEYS and isinstance(child, (str, bool)):
                    selections['.'.join(path + (key,))] = child
                elif isinstance(child, (dict, list)):
                    select(child, path + (key,))
        elif isinstance(value, list):
            for index, child in enumerate(value):
                select(child, path + (str(index),))
    select(doc)
    return {'path': relative, 'site': site, 'site_from_runtime_or_town_only': True,
            'site_identifiers_not_live_map_proof': sorted(site_identifiers),
            'split': doc.get('dataset_split', doc.get('split')),
            'status': doc.get('status') if isinstance(doc.get('status'), str) else None,
            'known_DEV_site_reused': doc.get('known_DEV_site_reused') is True,
            'control_source': doc.get('control_source') if isinstance(doc.get('control_source'), str) else None,
            'selection_metadata': selections, 'matches': matches}


def verify_allowlist(doc):
    rows = doc.get('metadata') if isinstance(doc, dict) else None
    if not isinstance(doc, dict) or doc.get('schema') != 'portable_e2e.rte01_good6000_named_metadata_allowlist.20261005.v2' or not isinstance(rows, list) or not 1 <= len(rows) <= MAX_FILES:
        raise ValueError('named metadata allowlist schema/count')
    paths = []
    total = 0
    for row in rows:
        if not isinstance(row, dict) or set(row) != {'path', 'sha256', 'bytes'} or not metadata_path_allowed(row['path']):
            raise ValueError('nonmetadata or unscoped allowlist path')
        require_sha(row['sha256'])
        if type(row['bytes']) is not int or not 0 <= row['bytes'] <= MAX_FILE_BYTES:
            raise ValueError('metadata byte count')
        paths.append(row['path'])
        total += row['bytes']
    if paths != sorted(set(paths)) or total > MAX_TOTAL_BYTES or doc.get('metadata_bytes') != total:
        raise ValueError('allowlist order/duplicate/total-byte bound')
    return rows


def expected_authorization(allowlist_sha):
    return {'schema': SCHEMA + '.authorization',
            'provenance': 'HH_261005 - Authorize only sealed canonical metadata projection for the exact GOOD6000 exposure ledger.',
            'model_weights_sha256': IDENTITIES['GOOD6000_weights'],
            'model_checkpoint_sha256': IDENTITIES['GOOD6000_checkpoint'],
            'named_metadata_allowlist_sha256': require_sha(allowlist_sha),
            'metadata_read_and_hash_allowed': True, 'TEST_payload_access_allowed': False,
            'TEST_NN_loss_labels_or_selection_allowed': False, 'training_allowed': False,
            'CARLA_GPU_SSH_installation_allowed': False, 'old_split_guard_modified': False,
            'V1_qualification_preserved': True, 'automatic_independent_TEST_approval': False,
            'coverage_scope': 'named Oct 1-5 archived metadata snapshot only; no claim of undocumented-history absence'}


def audit(allowlist, *, root=ROOT):
    rows = verify_allowlist(allowlist)
    records, errors, index_mentions = [], [], []
    for row in rows:
        data = read_regular(root / row['path'])
        if sha(data) != row['sha256'] or len(data) != row['bytes']:
            raise ValueError('sealed metadata changed: ' + row['path'])
        if row['path'].endswith('.md'):
            text = data.decode('utf-8')
            index_mentions.append({'path': row['path'], 'identity_tokens_mentioned':
                sorted(name for name, token in IDENTITIES.items() if token in text),
                'reserved_site_names_mentioned': [site for site in RESERVED_SITES if site in text],
                'mentions_are_not_runtime_or_training_binding': True})
            continue
        try:
            records.append(extract_record(row['path'], parse_json(data)))
        except (ValueError, UnicodeError) as exc:
            errors.append({'path': row['path'], 'error_type': type(exc).__name__,
                           'metadata_sha256': row['sha256'], 'treated_as_unknown_not_PASS': True})
    current = [r for r in records if any(m['identity'].startswith('GOOD6000_') and
                m['kind'] == 'direct_model_binding' for m in r['matches'])]
    all_matches = [r for r in records if r['matches']]
    observed = [r for r in current if r['path'].endswith('/manifest.json')]
    ancestral_diagnostics = [r for r in records if
        any(r['path'].startswith(prefix) for prefix in ANCESTRAL_RUNS) and
        Path(r['path']).name in EVALUATION_METADATA_NAMES | {'declaration.json', 'summary.json'}]
    declared_sites = sorted({r['site'] for r in current if r['site'] is not None})
    observed_sites = sorted({r['site'] for r in observed if r['site'] is not None})
    return {'schema': SCHEMA, 'created_UTC': datetime.now(timezone.utc).isoformat(),
            'status': 'BOUNDED_METADATA_LEDGER_NOT_TEST_APPROVAL',
            'metadata_count': len(rows), 'metadata_bytes': sum(r['bytes'] for r in rows),
            'all_named_metadata_bytes_source_pinned': True,
            'metadata_projection_parse_complete': not errors,
            'metadata_parse_errors_retained': errors, 'index_mentions': index_mentions,
            'current_GOOD6000_direct_binding_document_count': len(current),
            'current_GOOD6000_declared_sites': declared_sites,
            'current_GOOD6000_observed_manifest_sites': observed_sites,
            'current_GOOD6000_observed_episode_manifests': observed,
            'all_matching_metadata_including_parent_and_source_only': all_matches,
            'current_model_selection_and_declared_sites': current,
            'ancestral_run_linked_dataset_and_selection_metadata_not_loss_values': ancestral_diagnostics,
            'ancestral_diagnostic_link_is_run_provenance_not_independent_model_hash_proof': True,
            'reserved_site_direct_current_binding_found': {
                site: site in declared_sites for site in RESERVED_SITES},
            'reserved_site_independent_TEST_qualified': {site: False for site in RESERVED_SITES},
            'no_direct_match_means_only_no_match_in_this_named_snapshot': True,
            'global_undocumented_history_absence_claimed': False,
            'model_weights_or_remote_actual_ingestion_verified': False,
            'TEST_payload_labels_loss_values_NN_opened': False,
            'old_guard_or_roles_changed': False, 'formal_main_features_approved': 0,
            'formal_subfeatures_approved': 0}


def save_new(path, value):
    path = Path(path)
    if any(p.is_symlink() for p in (path, *path.parents)):
        raise ValueError('symlink output path')
    data = (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(data)
    return {'path': str(path), 'sha256': sha(data), 'created': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-authorization-sha256', required=True)
    parser.add_argument('--save-create-only', action='store_true')
    args = parser.parse_args(argv)
    expected = require_sha(args.expected_authorization_sha256)
    auth_data = read_regular(AUTHORIZATION)
    if sha(auth_data) != expected:
        raise ValueError('authorization source pin mismatch')
    allow_data = read_regular(ALLOWLIST)
    auth = parse_json(auth_data)
    if canonical(auth) != canonical(expected_authorization(sha(allow_data))):
        raise ValueError('authorization must preserve exact metadata-only denials')
    before = sha(read_regular(ROOT / GUARD))
    if before != GUARD_SHA:
        raise ValueError('original site guard pin changed')
    result = audit(parse_json(allow_data))
    after = sha(read_regular(ROOT / GUARD))
    if after != before:
        raise ValueError('original site guard changed during audit')
    result['authorization_sha256'] = expected
    result['named_allowlist_sha256'] = sha(allow_data)
    result['original_split_guard_sha256_before'] = before
    result['original_split_guard_sha256_after'] = after
    result['auditor_source_sha256'] = sha(read_regular(Path(__file__)))
    saved = save_new(OUTPUT, result) if args.save_create_only else {'created': False}
    print(json.dumps({'status': result['status'], 'current_declared_sites':
        result['current_GOOD6000_declared_sites'], 'current_observed_sites':
        result['current_GOOD6000_observed_manifest_sites'], 'parse_errors':
        len(result['metadata_parse_errors_retained']), 'output': saved}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
