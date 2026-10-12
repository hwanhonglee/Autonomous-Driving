"""HH_261001 - Recount fixed evidence without pickle loads, extra forward passes or changed probes."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest

from scripts.e2e import report_warmup_duration_w2_comparison_20261001 as report


def test_npz_numeric_hash_matches_original_tensor_byte_definition(tmp_path):
    values = np.array([1., -2.], dtype=np.float32)
    path = tmp_path / 'weights.npz'
    np.savez(path, a=values)
    metadata = dict(weights_sha256=report.sha(path), tensors={'a': dict(dtype='float32', shape=[2])})
    header = b'["a","torch.float32",[2]]'
    expected = hashlib.sha256(len(header).to_bytes(8, 'big') + header + values.tobytes()).hexdigest()
    assert report.npz_state_digest(path, metadata) == expected
    metadata['tensors']['a']['shape'] = [1, 2]
    with pytest.raises(ValueError):
        report.npz_state_digest(path, metadata)


@pytest.mark.parametrize('values', [np.array([np.nan], dtype=np.float32), np.array([{}], dtype=object), np.array([1.], dtype=np.float64)])
def test_unsafe_or_nonfinite_export_rejected(tmp_path, values):
    path = tmp_path / 'weights.npz'
    np.savez(path, a=values)
    metadata = dict(weights_sha256=report.sha(path), tensors={'a': dict(dtype=str(values.dtype), shape=[1])})
    with pytest.raises(ValueError):
        report.npz_state_digest(path, metadata)


def test_actual_fixed_w1_export_digest_without_checkpoint_deserialization():
    root = Path(__file__).resolve().parents[2]
    path = root / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/exports/W1_warmup_anchor_off.npz'
    metadata = json.loads(path.with_suffix('.json').read_text())
    assert report.npz_state_digest(path, metadata) == report.W1_STATE


def test_first_block_exact_identity_and_updates_required():
    first = dict(status='PASS_EXACT_W1_STATE', expected_model_state_sha256=report.W1_STATE,
        actual_model_state_sha256=report.W1_STATE, fit=dict(report.FIRST_FIT), optimizer_same_object=True,
        optimizer_reloaded=False, checkpoint_used_for_model_selection=False, checkpoint='first_block_identity_checkpoint.pt')
    report.verify_first(first)
    first['fit']['global_step'] = 0
    with pytest.raises(ValueError):
        report.verify_first(first)


def test_existing_fixed_279_probes_compare_to_self_without_new_selection():
    root = Path(__file__).resolve().parents[2] / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1'
    declaration = json.loads((root / 'declaration.json').read_text())
    proof = json.loads((root / 'provenance_receipt.json').read_text())
    rows = json.loads((root / report.W1 / 'train_probe_after.json').read_text())
    models, changes = report.compare_probes(rows, deepcopy(rows), declaration, proof)
    assert models['W1']['groups'] == models['W2']['groups']
    assert models['W1']['groups']['original_immediate_start18']['count'] == 18
    assert models['W1']['groups']['protected_stop_hold200']['count'] == 200
    assert all(row['oracle_agreement_change'] == row['near_speed_mae_delta_mps'] == 0 for row in changes)
    counts = report.change_counts(changes)
    assert counts['immediate_start18']['early_speed_mae_equal'] == 18
    assert counts['stop_hold200']['oracle_match_lost'] == 0
    altered = deepcopy(rows)
    altered[0]['target_speed'][0] += .1
    with pytest.raises(ValueError):
        report.compare_probes(rows, altered, declaration, proof)


def test_existing_dev_gates_recompute_and_mismatch_rejected():
    root = Path(__file__).resolve().parents[2] / 'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1'
    evaluation = json.loads((root / report.W1 / 'final_dev_evaluation.json').read_text())
    gates = json.loads((root / 'summary.json').read_text())['strict_gates'][report.W1]
    rows = report.compare_metrics(evaluation, evaluation, gates, gates)
    assert all(row['W2_minus_W1'] == 0 for row in rows)
    changed = deepcopy(gates)
    changed['metric_checks'][0]['maximum'] += 1.
    with pytest.raises(ValueError):
        report.compare_metrics(evaluation, evaluation, gates, changed)
