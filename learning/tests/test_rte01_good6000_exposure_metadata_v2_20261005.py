"""HH_261005 - Keep metadata exposure distinct from inference, training and TEST approval."""
import importlib.util
import json
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[2] / 'tools/e2e/audit_rte01_good6000_exposure_metadata_v2_20261005.py'
SPEC = importlib.util.spec_from_file_location('good6000_metadata_v2_tests', PATH)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def allowlist(rows):
    rows = sorted(rows, key=lambda row: row['path'])
    return {'schema': 'portable_e2e.rte01_good6000_named_metadata_allowlist.20261005.v2',
            'metadata': rows, 'metadata_bytes': sum(row['bytes'] for row in rows)}


def enroll(tmp_path, relative, document, *, raw=None):
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    data = raw if raw is not None else json.dumps(document).encode()
    path.write_bytes(data)
    return {'path': relative, 'bytes': len(data), 'sha256': m.sha(data)}


@pytest.mark.parametrize('relative', [
    '/tmp/manifest.json', '../metadata.json', 'storage/experiments/validation/2026-09-30/run/manifest.json',
    'storage/experiments/validation/2026-10-04/run/states.jsonl',
    'storage/experiments/validation/2026-10-04/run/states.json',
    'storage/experiments/plans/2026-10-04/run/labels.json',
    'storage/experiments/plans/2026-10-04/run/samples.json',
    'storage/experiments/validation/2026-10-04/run/images/manifest.json',
    'storage/experiments/validation/2026-10-04/run/source/metadata.json',
    'storage/experiments/validation/2026-10-04/run/provenance/metadata.json',
    'storage/experiments/validation/2026-10-04/run/derived_body/summary.json',
    'storage/experiments/validation/2026-10-04/run/episode/moving_task_plan.json',
    'storage/experiments/training/2026-10-04/run/model_state.npz',
    'storage/experiments/training/2026-10-04/run/checkpoint.pt',
])
def test_nonmetadata_or_unscoped_payload_paths_rejected(relative):
    assert not m.metadata_path_allowed(relative)


@pytest.mark.parametrize('relative', [
    'storage/experiments/validation/2026-10-04/run/episode.partial/manifest.json',
    'storage/experiments/validation/2026-10-04/run/episode/learned_summary.json',
    'storage/experiments/validation/2026-10-04/run_plan.json',
    'storage/experiments/training/2026-10-02/run/exports/step_006000/metadata.json',
    'storage/experiments/training/2026-10-02/run/known599_final.json',
    'storage/experiments/diagnostics/2026-10-04/run/summary.json',
    'storage/experiments/plans/2026-10-04/run/ROOT_TRAIN_only_authorization.json',
    m.GUARD,
])
def test_narrow_named_metadata_paths_allowed(relative):
    assert m.metadata_path_allowed(relative)


def test_real_direct_identity_not_substring_or_other_checkpoint():
    record = m.extract_record('manifest.json', {'runtime': {'town': 'Town01'},
        'capture_contract': {'learned_research': {'checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint']}},
        'source_checkpoint_sha256': 'f' * 64, 'note': 'prefix' + m.IDENTITIES['GOOD6000_weights']})
    assert record['site'] == 'Town01'
    assert len(record['matches']) == 1
    assert record['matches'][0]['kind'] == 'direct_model_binding'


@pytest.mark.parametrize('field', ['asset_sha256', 'source_sha256', 'immutable_input_sha256', 'source_hashes'])
def test_source_pin_does_not_prove_current_model_exposure(field):
    record = m.extract_record('declaration.json', {field: {'/archive/model_state.npz': m.IDENTITIES['GOOD6000_weights']}, 'town': 'Town15'})
    assert record['matches'][0]['kind'] == 'source_archive_reference_not_active_model'


@pytest.mark.parametrize('field', ['parent', 'actual_parent_final6000', 'ancestry'])
def test_descendant_parent_is_not_same_candidate_inference(field):
    record = m.extract_record('metadata.json', {field: {'checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint']}})
    assert record['matches'][0]['kind'] == 'ancestry_or_descendant_parent_reference'


def test_numeric_losses_raw_predictions_and_prefill_payload_never_projected():
    record = m.extract_record('summary.json', {
        'losses': {'checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint'], 'value': 123.0},
        'raw_predictions': {'checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint']},
        'prefill_records': [{'checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint']}],
        'checkpoint_selection_by_holdout': False, 'selection_rule': 'fixed final6000',
    })
    assert record['matches'] == []
    assert record['selection_metadata'] == {'checkpoint_selection_by_holdout': False, 'selection_rule': 'fixed final6000'}
    assert '123' not in json.dumps(record)


def test_runtime_site_not_inferred_from_misleading_dataset_directory():
    record = m.extract_record('metadata.json', {'path': '/prepared/B_town07_augmented_train', 'episode_id': 'carla_town03_case'})
    assert record['site'] is None
    assert record['site_identifiers_not_live_map_proof'] == ['town03']


def test_actual_failed_manifest_preserved_without_function_or_TEST_pass(tmp_path):
    rows = [enroll(tmp_path, 'storage/experiments/validation/2026-10-02/run/episode.partial/manifest.json',
        {'runtime': {'town': 'Town01'}, 'status': 'failed',
         'capture_contract': {'learned_research': {'checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint']}}}),
        enroll(tmp_path, 'storage/experiments/validation/2026-10-04/sourceonly/declaration.json',
         {'town': 'Town15', 'asset_sha256': {'/old/model_state.npz': m.IDENTITIES['GOOD6000_weights']}})]
    result = m.audit(allowlist(rows), root=tmp_path)
    assert result['current_GOOD6000_declared_sites'] == ['Town01']
    assert result['current_GOOD6000_observed_manifest_sites'] == ['Town01']
    assert result['current_GOOD6000_observed_episode_manifests'][0]['status'] == 'failed'
    assert result['reserved_site_direct_current_binding_found']['Town15'] is False
    assert not any(result['reserved_site_independent_TEST_qualified'].values())
    assert result['formal_main_features_approved'] == result['formal_subfeatures_approved'] == 0


def test_actual_reserved_current_binding_detected_but_never_auto_approved(tmp_path):
    row = enroll(tmp_path, 'storage/experiments/validation/2026-10-04/example/declaration.json',
        {'town': 'Town04', 'source_checkpoint_sha256': m.IDENTITIES['GOOD6000_checkpoint']})
    result = m.audit(allowlist([row]), root=tmp_path)
    assert result['reserved_site_direct_current_binding_found']['Town04'] is True
    assert result['reserved_site_independent_TEST_qualified']['Town04'] is False


def test_bad_metadata_is_retained_unknown_not_silently_skipped(tmp_path):
    row = enroll(tmp_path, 'storage/experiments/validation/2026-10-01/old/summary.json', None, raw=b'{bad-json')
    result = m.audit(allowlist([row]), root=tmp_path)
    assert result['metadata_projection_parse_complete'] is False
    assert result['metadata_parse_errors_retained'][0]['treated_as_unknown_not_PASS'] is True


@pytest.mark.parametrize('raw', [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":Infinity}'])
def test_ambiguous_or_nonfinite_json_rejected(raw):
    with pytest.raises(ValueError):
        m.parse_json(raw)


@pytest.mark.parametrize('mutation', ['duplicate', 'bad_sha', 'bool_size', 'wrong_total', 'too_large', 'payload'])
def test_malformed_allowlist_denied(mutation):
    row = {'path': 'storage/experiments/validation/2026-10-04/run/manifest.json', 'sha256': 'a' * 64, 'bytes': 1}
    doc = allowlist([row])
    if mutation == 'duplicate': doc['metadata'].append(dict(row))
    elif mutation == 'bad_sha': row['sha256'] = 'a' * 63
    elif mutation == 'bool_size': row['bytes'] = True
    elif mutation == 'wrong_total': doc['metadata_bytes'] = 99
    elif mutation == 'too_large': row['bytes'] = m.MAX_FILE_BYTES + 1
    elif mutation == 'payload': row['path'] = 'storage/experiments/plans/2026-10-04/run/states.json'
    with pytest.raises(ValueError): m.verify_allowlist(doc)


@pytest.mark.parametrize('value', [None, [], {}, {'metadata': []}])
def test_invalid_allowlist_container_rejected(value):
    with pytest.raises(ValueError): m.verify_allowlist(value)


def test_named_file_hash_mismatch_and_symlink_fail_closed(tmp_path):
    row = enroll(tmp_path, 'storage/experiments/validation/2026-10-04/run/manifest.json', {})
    doc = allowlist([row])
    (tmp_path / row['path']).write_bytes(b'[]')
    with pytest.raises(ValueError, match='changed'): m.audit(doc, root=tmp_path)
    link = tmp_path / 'link'
    link.symlink_to(tmp_path / row['path'])
    with pytest.raises(ValueError, match='symlink'): m.read_regular(link)


def test_create_only_results_refuse_overwrite_and_symlink(tmp_path):
    output = tmp_path / 'result.json'
    assert m.save_new(output, {'failed': True})['created'] is True
    with pytest.raises(FileExistsError): m.save_new(output, {'failed': False})
    assert json.loads(output.read_text()) == {'failed': True}
    link = tmp_path / 'link.json'
    link.symlink_to(output)
    with pytest.raises(ValueError): m.save_new(link, {})


def test_authorization_denies_payload_execution_and_role_changes():
    auth = m.expected_authorization('a' * 64)
    assert auth['metadata_read_and_hash_allowed'] is True
    for key in ('TEST_payload_access_allowed', 'TEST_NN_loss_labels_or_selection_allowed',
                'training_allowed', 'CARLA_GPU_SSH_installation_allowed',
                'old_split_guard_modified', 'automatic_independent_TEST_approval'):
        assert auth[key] is False
    assert auth['V1_qualification_preserved'] is True


def test_ancestral_diagnostic_provenance_retained_without_loss_values(tmp_path):
    path = m.ANCESTRAL_RUNS[-1] + 'known599_final.json'
    row = enroll(tmp_path, path, {'per_episode': [{'episode_id': 'carla_town03_example', 'loss': 99.0}]})
    result = m.audit(allowlist([row]), root=tmp_path)
    diagnostic = result['ancestral_run_linked_dataset_and_selection_metadata_not_loss_values'][0]
    assert diagnostic['site_identifiers_not_live_map_proof'] == ['town03']
    assert '99.0' not in json.dumps(result)
    assert result['current_GOOD6000_direct_binding_document_count'] == 0
