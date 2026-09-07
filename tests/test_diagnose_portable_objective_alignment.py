"""HH_260906 - Test exact training-oracle reuse, original masks, provenance, and read-only scope."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from portable_e2e.contract import ContractError
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('objective_diagnostic',
    ROOT / 'scripts/e2e/diagnose_portable_objective_alignment.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def predictions():
    # HH_260906 - Candidate zero wins ADE, candidate one wins composite loss, and logits select candidate two.
    xy = torch.zeros(2, 6, 3, 2)
    xy[:, :, :, 1] = torch.arange(6, dtype=torch.float32)[None, :, None]
    speed = torch.zeros(2, 6, 3)
    speed[:, 0] = 20
    logits = torch.zeros(2, 6)
    logits[:, 2] = 1
    return dict(xy=xy, speed=speed, logits=logits, target=torch.zeros(2, 3, 2),
        target_speed=torch.zeros(2, 3), valid=torch.ones(2, 3, dtype=torch.bool),
        target_yaw=torch.zeros(2, 3), loss_config=TrajectoryLossConfig(speed_weight=1,
            yaw_weight=0, kinematic_speed_weight=0, final_displacement_weight=0))


def test_three_distinct_decisions_and_exact_training_loss_reuse(monkeypatch):
    values = predictions()
    snapshots = {key: value.clone() for key, value in values.items() if isinstance(value, torch.Tensor)}
    calls = []

    def actual_loss(*args, **kwargs):
        calls.append((args, kwargs))
        return trajectory_loss(*args, **kwargs)

    monkeypatch.setattr(MODULE, 'trajectory_loss', actual_loss)
    result = MODULE.summarize_alignment(**values)
    assert len(calls) == 1
    assert calls[0][0][5] is values['valid']
    assert calls[0][0][6] is values['loss_config']
    assert calls[0][1]['target_yaw'] is values['target_yaw']
    assert result['selected_histogram'] == [0, 0, 2, 0, 0, 0]
    assert result['composite_oracle_histogram'] == [0, 2, 0, 0, 0, 0]
    assert result['ade_oracle_histogram'] == [2, 0, 0, 0, 0, 0]
    for pair in ('selected_composite_oracle', 'selected_ade_oracle', 'composite_oracle_ade_oracle'):
        assert result[f'{pair}_agreement_count'] == 0
        assert sum(map(sum, result[f'{pair}_confusion_rows_left_columns_right'])) == 2
    assert result['mean_selected_ade_m'] == 2
    assert result['mean_composite_oracle_ade_m'] == 1
    assert result['mean_ade_oracle_ade_m'] == 0
    for key, original in snapshots.items():
        assert torch.equal(values[key], original)


def test_original_valid_prefix_masks_all_losses_and_ade():
    values = predictions()
    values['valid'][:, -1] = False
    baseline = MODULE.summarize_alignment(**values)
    for key in ('target', 'target_speed', 'target_yaw'):
        values[key][:, -1] = 10000
    values['xy'][:, :, -1] = 1000
    values['speed'][:, :, -1] = 10000
    result = MODULE.summarize_alignment(**values)
    assert result == baseline
    assert result['per_sample'][0]['target_valid'] == [True, True, False]
    assert result['per_sample'][0]['valid_point_count'] == 2


def test_default_and_half_score_weights_do_not_redefine_regression_oracle():
    values = predictions()
    first = MODULE.summarize_alignment(**values)
    values['loss_config'] = TrajectoryLossConfig(**{**values['loss_config'].to_dict(), 'candidate_score_weight': 0.5})
    assert MODULE.summarize_alignment(**values) == first


def test_exact_ties_use_first_index_and_all_agreements_have_full_denominator():
    values = predictions()
    values['xy'].zero_()
    values['speed'].zero_()
    values['logits'].zero_()
    result = MODULE.summarize_alignment(**values)
    assert result['selected_histogram'] == result['composite_oracle_histogram'] == result['ade_oracle_histogram']
    assert result['selected_histogram'] == [2, 0, 0, 0, 0, 0]
    assert result['selected_composite_oracle_agreement_rate'] == 1
    assert result['mean_selected_minus_composite_oracle_ade_m'] == 0


@pytest.mark.parametrize('fault', ['nonprefix', 'empty', 'integer_mask', 'nan_speed', 'nan_yaw', 'missing_yaw'])
def test_invalid_original_inputs_fail_closed(fault):
    values = predictions()
    if fault == 'nonprefix':
        values['valid'][0] = torch.tensor([True, False, True])
    elif fault == 'empty':
        values['valid'][0] = False
    elif fault == 'integer_mask':
        values['valid'] = values['valid'].long()
    elif fault == 'nan_speed':
        values['speed'][0, 0, 0] = float('nan')
    elif fault == 'nan_yaw':
        values['target_yaw'][0, 0] = float('nan')
    else:
        values['target_yaw'] = None
    with pytest.raises((ContractError, ValueError, FloatingPointError)):
        MODULE.summarize_alignment(**values)


@pytest.mark.parametrize('value', [None, {}, {'xy_weight': 1}, {**TrajectoryLossConfig().to_dict(), 'extra': 0},
    {**TrajectoryLossConfig().to_dict(), 'candidate_score_weight': -1}])
def test_loss_config_must_be_complete_and_valid(value):
    with pytest.raises(ContractError):
        MODULE.checkpoint_loss_config({'loss_config': value})


def test_checkpoint_loss_config_is_not_defaulted_or_modified():
    value = {**TrajectoryLossConfig().to_dict(), 'candidate_score_weight': 0.5}
    assert MODULE.checkpoint_loss_config({'loss_config': value}).to_dict() == value


class FrozenDataset:
    def __init__(self, split):
        self.split = split
        self.fingerprint_sha256 = MODULE.SPLITS[split][1]
        self.examples = [SimpleNamespace(episode_id='training' if split == 'train' else 'validation')]

    def __len__(self):
        return MODULE.SPLITS[self.split][0]


def test_train_is_exact_in_sample_split_not_mislabeled_validation():
    train = FrozenDataset('train')
    payload = {'dataset_fingerprint_sha256': train.fingerprint_sha256}
    MODULE.validate_split(train, 'train', payload, ('training',))
    MODULE.validate_split(FrozenDataset('val'), 'val', payload, ('training',))
    with pytest.raises(ContractError, match='episode'):
        MODULE.validate_split(train, 'train', payload, ('other',))
    with pytest.raises(ContractError, match='fingerprint'):
        MODULE.validate_split(train, 'train', {'dataset_fingerprint_sha256': '0' * 64}, ('training',))
    with pytest.raises(ContractError, match='leakage'):
        MODULE.validate_split(FrozenDataset('val'), 'val', payload, ('validation',))
    with pytest.raises(ContractError):
        MODULE.validate_split(train, 'test', payload, ('training',))
    train.fingerprint_sha256 = '0' * 64
    with pytest.raises(ContractError, match='entire frozen'):
        MODULE.validate_split(train, 'train', payload, ('training',))


def test_hash_verification_rejects_stale_malformed_and_symlink_inputs(tmp_path):
    path = tmp_path / 'input'
    path.write_bytes(b'actual input')
    digest = MODULE.file_sha256(path)
    assert MODULE.require_hash(path, digest) == digest
    for invalid in ('0' * 64, digest.upper(), None, 'no hash'):
        with pytest.raises(ContractError):
            MODULE.require_hash(path, invalid)
    link = tmp_path / 'link'
    link.symlink_to(path)
    with pytest.raises(ContractError):
        MODULE.require_hash(link, digest)


def test_cli_requires_all_provenance_pins_and_never_offers_test_or_gpu():
    base = ['dataset', '--checkpoint', 'model.pt', '--checkpoint-sha256', 'a' * 64,
        '--script-sha256', 'b' * 64, '--expected-source-commit', 'c' * 40, '--output-dir', 'output']
    assert MODULE.parse_args(base + ['--include-train']).include_train
    for flag in ('--checkpoint-sha256', '--script-sha256', '--expected-source-commit'):
        args = base.copy()
        index = args.index(flag)
        del args[index:index + 2]
        with pytest.raises(SystemExit):
            MODULE.parse_args(args)
    for forbidden, value in (('--split', 'test'), ('--device', 'cuda:0'), ('--limit-samples', '1')):
        with pytest.raises(SystemExit):
            MODULE.parse_args(base + [forbidden, value])


def test_cpu_environment_and_existing_output_rejected_before_any_load(monkeypatch, tmp_path):
    args = (tmp_path, tmp_path / 'model.pt', 'a' * 64, tmp_path / 'output', 'b' * 64, 'c' * 40)
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES', '0')
    with pytest.raises(ContractError, match='CUDA_VISIBLE_DEVICES'):
        MODULE.diagnose(*args)
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES', '')
    monkeypatch.setenv('OMP_NUM_THREADS', '8')
    with pytest.raises(ContractError, match='thread limits'):
        MODULE.diagnose(*args)
    monkeypatch.setenv('OMP_NUM_THREADS', '4')
    monkeypatch.setenv('MKL_NUM_THREADS', '4')
    args[3].mkdir()
    with pytest.raises(ContractError, match='must be new'):
        MODULE.diagnose(*args)


def test_batch_reader_uses_real_dataset_speed_and_yaw_keys(monkeypatch):
    values = predictions()
    batch = {key: torch.zeros(2, 1) for key in ('images', 'calibration', 'ego_history',
        'ego_history_mask', 'route_xy', 'route_mask')}
    batch.update(sample_id=['sample_0', 'sample_1'], target_xy=values['target'],
        target_speed_mps=values['target_speed'], target_yaw_rad=values['target_yaw'],
        target_valid=values['valid'])
    monkeypatch.setattr(MODULE, 'DataLoader', lambda *args, **kwargs: [batch])
    dataset = FrozenDataset('val')
    monkeypatch.setattr(FrozenDataset, '__len__', lambda self: 2)
    model = lambda *args: (values['xy'], values['speed'], values['logits'])
    report = MODULE.analyze_split(model, dataset, values['loss_config'])
    assert report['summary']['composite_oracle_histogram'] == [0, 2, 0, 0, 0, 0]
    assert [row['sample_id'] for row in report['summary']['per_sample']] == batch['sample_id']
    batch['sample_id'] = ['sample_0', 'sample_0']
    with pytest.raises(ContractError, match='duplicated'):
        MODULE.analyze_split(model, dataset, values['loss_config'])


@pytest.mark.parametrize('fault', ['commit', 'dirty', 'wrong_root', None])
def test_imported_source_requires_exact_clean_commit(monkeypatch, fault):
    expected = 'c' * 40

    def fake_git(command, **kwargs):
        if '--show-toplevel' in command:
            output = str(ROOT / 'wrong') if fault == 'wrong_root' else str(ROOT)
        elif 'HEAD' in command:
            output = 'd' * 40 if fault == 'commit' else expected
        else:
            output = '?? untracked.py' if fault == 'dirty' else ''
        return SimpleNamespace(stdout=output)

    monkeypatch.setattr(MODULE.subprocess, 'run', fake_git)
    if fault:
        with pytest.raises(ContractError):
            MODULE.source_provenance(expected)
    else:
        result = MODULE.source_provenance(expected)
        assert result['diagnostic_source_commit'] == expected
        assert 'portable_e2e/losses.py' in result['source_sha256']
        assert 'scripts/e2e/diagnose_portable_selector.py' in result['source_sha256']


@pytest.mark.parametrize('source_changes', [False, True])
def test_new_only_report_keeps_train_val_separate_and_rechecks_before_writing(monkeypatch, tmp_path, source_changes):
    # HH_260906 - Exercise orchestration without loading pickle or reading test examples in unit tests.
    for name, value in [('CUDA_VISIBLE_DEVICES', ''), ('OMP_NUM_THREADS', '4'), ('MKL_NUM_THREADS', '4')]:
        monkeypatch.setenv(name, value)
    monkeypatch.setattr(MODULE.torch, 'set_num_threads', lambda _: None)
    monkeypatch.setattr(MODULE.torch, 'set_num_interop_threads', lambda _: None)
    hashes = []
    monkeypatch.setattr(MODULE, 'require_hash', lambda path, expected: hashes.append((Path(path).name, expected)))
    source_calls = []

    def source(commit):
        source_calls.append(commit)
        return {'diagnostic_source_commit': commit,
            'source_sha256': {'losses.py': 'a' * 64 if len(source_calls) == 1 or not source_changes else 'b' * 64}}

    monkeypatch.setattr(MODULE, 'source_provenance', source)
    loaded_splits = []

    def load(root, *, split, **kwargs):
        loaded_splits.append(split)
        return SimpleNamespace(examples=split,
            validation_report={'dataset_fingerprint_sha256': MODULE.CORPUS_SHA256})

    monkeypatch.setattr(MODULE, 'load_training_examples', load)
    payload = {'loss_config': TrajectoryLossConfig().to_dict(),
        'dataset_fingerprint_sha256': MODULE.SPLITS['train'][1]}
    monkeypatch.setattr(MODULE, '_read_checkpoint_for_audit', lambda **kwargs: (payload, None, ('training',), {}))
    monkeypatch.setattr(MODULE, 'Common10TorchDataset', lambda examples, config, **kwargs: FrozenDataset(kwargs['split']))
    validated_model_splits = []

    def validate(dataset, **kwargs):
        validated_model_splits.append(dataset.split)
        return object(), {'checkpoint_sha256': 'a' * 64}

    monkeypatch.setattr(MODULE, '_validate_checkpoint_and_model', validate)
    monkeypatch.setattr(MODULE, 'analyze_split', lambda model, dataset, config:
        {'split': dataset.split, 'summary': {'sample_count': len(dataset)}})
    output = tmp_path / 'new_report'
    args = (tmp_path, tmp_path / 'model.pt', 'a' * 64, output, 'b' * 64, 'c' * 40)
    if source_changes:
        with pytest.raises(ContractError, match='source changed'):
            MODULE.diagnose(*args, include_train=True)
        assert not output.exists()
    else:
        report = MODULE.diagnose(*args, include_train=True)
        assert (output / 'objective_alignment.json').is_file()
        assert set(report['splits']) == {'val', 'train'}
        assert report['splits']['val']['summary']['sample_count'] == 337
        assert report['splits']['train']['summary']['sample_count'] == 1147
        assert report['vehicle_control_approved'] is False
        with pytest.raises(ContractError, match='must be new'):
            MODULE.diagnose(*args)
    assert loaded_splits == ['val', 'train']
    assert validated_model_splits == ['val']
    assert hashes.count(('dataset.json', MODULE.MANIFEST_SHA256)) == 2
    assert hashes.count(('model.pt', 'a' * 64)) == 2
