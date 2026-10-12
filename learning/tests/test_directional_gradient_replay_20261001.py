"""HH_261001 - Fixed directional pair identity, unmodified control/reserves and ownership composition."""

from copy import deepcopy
from types import SimpleNamespace
import pytest

from scripts.e2e import research_directional_gradient_replay_20261001 as study


def capture(arm='A'):
    return SimpleNamespace(host='127.0.0.1', port=2440, allow_map_load=False,
        control_transport='acknowledged_batch', goal_stop_profile='disabled', physics_hz=20, capture_hz=10,
        stationary_warmup_sec=1.2, max_duration_sec=20, research_speed_cap_mps=2.5,
        target_speed_kmh=9, research_max_cte_m=1., weather='ClearNoon', seed=0,
        output=study.output(arm) / 'episode')


def fake_endpoints():
    return {arm: dict(checkpoint_sha256=study.CHECKPOINTS[arm], model_state_sha256=study.STATES[arm],
        checkpoint_schema=study.proof.SCHEMA, training_arm=training_arm,
        weights=str(study.TRAINING / 'exports' / (training_arm + '.npz')),
        metadata=str(study.TRAINING / 'exports' / (training_arm + '.json')),
        weights_sha256='1' * 64, metadata_sha256='2' * 64)
        for arm, training_arm in zip(study.proof.LABELS, study.proof.ARMS)}


@pytest.mark.parametrize('arm', ['A', 'B'])
def test_identity_is_actual_new_checkpoint_and_restores_after_failure(arm):
    saved = study.research.AH_CHECKPOINT_SHA256, study.research.CHECKPOINT_SCHEMA
    collector = SimpleNamespace(AH_CHECKPOINT_SHA256='old')
    with pytest.raises(RuntimeError):
        with study.identity(arm, study.CHECKPOINTS[arm], collector):
            assert study.research.AH_CHECKPOINT_SHA256 == collector.AH_CHECKPOINT_SHA256 == study.CHECKPOINTS[arm]
            assert study.research.CHECKPOINT_SCHEMA == study.proof.SCHEMA
            raise RuntimeError('injected')
    assert (study.research.AH_CHECKPOINT_SHA256, study.research.CHECKPOINT_SCHEMA) == saved
    assert collector.AH_CHECKPOINT_SHA256 == 'old'


@pytest.mark.parametrize('arm,sha', [('A', study.prior.w1.CHECKPOINT), ('A', study.prior.PARENT_SHA),
    ('B', study.CHECKPOINTS['A']), ('W1', study.CHECKPOINTS['A'])])
def test_no_historical_or_parent_identity_substitution(arm, sha):
    with pytest.raises(ValueError):
        with study.identity(arm, sha):
            pass


@pytest.mark.parametrize('field,value', [('max_duration_sec', 21), ('target_speed_kmh', 30),
    ('research_speed_cap_mps', 30 / 3.6), ('research_max_cte_m', 2), ('goal_stop_profile', 'comfortable_v2'),
    ('physics_hz', 10), ('capture_hz', 20), ('allow_map_load', True), ('stationary_warmup_sec', 1.3),
    ('control_transport', 'direct')])
def test_unchanged_capture_contract_and_scoped_validator(field, value):
    old = study.prior.output
    for arm in ('A', 'B'):
        args = capture(arm)
        study.validate_capture(args, dict(town='Town07', scenario='straight'), arm)
        setattr(args, field, value)
        with pytest.raises(ValueError):
            study.validate_capture(args, dict(town='Town07', scenario='straight'), arm)
    assert study.prior.output is old


def test_exact_old_disk_reserves_and_sparse_observer_unchanged():
    from scripts.e2e import collect_w1_town07_long_20261001 as original
    assert study.prior.w1.sampled_inputs is original.sampled_inputs
    assert study.prior.disk_check(2, free=5972688896)['required_bytes'] == 5972688896
    assert study.prior.disk_check(1, free=5704253440)['required_bytes'] == 5704253440
    for arms, free in [(2, 5972688895), (1, 5704253439)]:
        with pytest.raises(ValueError):
            study.prior.disk_check(arms, free=free)


def test_runtime_floor_returns_no_camera_bundle(monkeypatch):
    calls = []
    original = study.short.collector.exact_camera_bundle
    monkeypatch.setattr(study.prior.shutil, 'disk_usage', lambda _: SimpleNamespace(free=study.prior.FLOOR))
    monkeypatch.setattr(study.short.collector, 'exact_camera_bundle', lambda *_: calls.append('called'))
    with pytest.raises(study.short.collector.CollectionError):
        with study.prior.disk_guard():
            study.short.collector.exact_camera_bundle(None)
    assert calls == []
    assert study.short.collector.exact_camera_bundle is not original


def test_real_both_endpoints_all_42_files_current_34_and_secure_npz(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError('local checkpoint deserialization forbidden')
    monkeypatch.setattr(study.research, '_read_checkpoint_file', forbidden)
    endpoints, pins = study.released_endpoints()
    study.torch.set_num_threads(2)
    for arm, item in endpoints.items():
        model = study.load_endpoint(arm, item)
        assert type(model).__module__ == 'portable_e2e.model'
        assert model._research_source_checkpoint_sha256 == study.CHECKPOINTS[arm]
        assert study.prior.tensor_digest(model.state_dict()) == study.STATES[arm]
    assert len(study.read_json(study.BINDING)['evidence_file_sha256']) == 42
    assert len(study.read_json(study.TRAINING / 'declaration.json')['source_sha256']) == 34
    study.source_unchanged(pins)


def test_bad_binding_and_changed_current_source_fail_closed(monkeypatch):
    real = study.digest
    monkeypatch.setattr(study, 'digest', lambda path: '0' * 64 if path == study.BINDING else real(path))
    with pytest.raises(ValueError, match='binding bytes'):
        study.released_endpoints()
    target = study.ROOT / 'learning/portable_e2e/model.py'
    monkeypatch.setattr(study, 'digest', lambda path: '0' * 64 if path == target else real(path))
    with pytest.raises(ValueError, match='source changed'):
        study.released_endpoints()


def write_a_fixture(folder, **changes):
    result = dict(port_triplet_released=True, source_hashes_match=True, owner_cleanup_errors=[],
        status='CAPTURE_FAILED_RETAINED', progress_m=0, arm='A', source_checkpoint_sha256=study.CHECKPOINTS['A'])
    result.update(changes)
    study.write_new(folder / 'owner_diagnostic_result.json', result)
    study.write_new(folder / 'owner_plan.json', dict(binding_sha256=study.BINDING_SHA,
        replay_plan_sha256=study.PLAN_SHA, arm='A', schema=study.SCHEMA,
        source_checkpoint_sha256=study.CHECKPOINTS['A'], source_sha256={}))


def test_predecessor_checks_cleanup_not_motion_outcome(monkeypatch, tmp_path):
    monkeypatch.setattr(study, 'output', lambda _: tmp_path)
    write_a_fixture(tmp_path)
    assert study.prior_a_ready()['status'] == 'CAPTURE_FAILED_RETAINED'


def test_predecessor_rejects_unsafe_cleanup(monkeypatch, tmp_path):
    monkeypatch.setattr(study, 'output', lambda _: tmp_path)
    write_a_fixture(tmp_path, owner_cleanup_errors=['owned process remains'])
    with pytest.raises(ValueError, match='cleanup required'):
        study.prior_a_ready()


@pytest.mark.parametrize('arm,reserved', [('A', 2), ('B', 1)])
def test_owner_preflight_no_launch_actual_identity_and_globals_restore(monkeypatch, tmp_path, arm, reserved):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    endpoints = fake_endpoints()
    pins = {study.Path(item[field]): item[field + '_sha256']
        for item in endpoints.values() for field in ('weights', 'metadata')}
    monkeypatch.setattr(study, 'released_endpoints', lambda: (endpoints, pins))
    monkeypatch.setattr(study, 'output', lambda arm: tmp_path / arm)
    monkeypatch.setattr(study.prior, 'disk_check', lambda arms=1: dict(reserved_arms=arms))
    monkeypatch.setattr(study, 'load_endpoint', lambda *_: None)
    checks = []
    monkeypatch.setattr(study, 'prior_a_ready', lambda: checks.append('A checked'))
    monkeypatch.setattr(owner, 'preflight', lambda: dict(collector_command=owner.collector_command()))
    def forbidden(*_args, **_kwargs):
        raise AssertionError('preflight launched simulator')
    monkeypatch.setattr(owner, 'execute', forbidden)
    saved = (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
             owner.write_json, owner.lifecycle._workspace_lock)
    result = study.trial(SimpleNamespace(arm=arm, mode='preflight'))
    command = result['collector_command']
    assert study.MODULE in command
    assert command[command.index('--arm') + 1] == arm
    assert command[command.index('--target-speed-kmh') + 1] == '9'
    assert command[command.index('--max-duration-sec') + 1] == '20'
    assert command[command.index('--goal-stop-profile') + 1] == 'disabled'
    assert '--research-weights' not in command
    assert result['source_checkpoint_sha256'] == study.CHECKPOINTS[arm]
    assert result['source_checkpoint_schema'] == study.proof.SCHEMA
    assert result['disk_reservation']['reserved_arms'] == reserved
    assert result['this_single_arm_is_not_a_pair'] is True
    assert checks == (['A checked'] if arm == 'B' else [])
    assert (owner.OUTPUT, owner.WEIGHTS, owner.METADATA, owner.PINS, owner.collector_command,
            owner.write_json, owner.lifecycle._workspace_lock) == saved


def test_disk_failure_precedes_simulator_and_leaves_no_output(monkeypatch, tmp_path):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    monkeypatch.setattr(study, 'released_endpoints', lambda: (fake_endpoints(), {}))
    monkeypatch.setattr(study, 'output', lambda arm: tmp_path / arm)
    monkeypatch.setattr(study.prior.shutil, 'disk_usage', lambda _: SimpleNamespace(free=study.prior.FLOOR))
    def forbidden(*_args, **_kwargs):
        raise AssertionError('disk failure launched simulator')
    monkeypatch.setattr(owner, 'execute', forbidden)
    with pytest.raises(ValueError, match='reserve unavailable'):
        study.trial(SimpleNamespace(arm='A', mode='preflight'))
    assert not (tmp_path / 'A').exists()


def test_single_attempt_cannot_be_counted_as_pair(monkeypatch, tmp_path):
    monkeypatch.setattr(study, 'released_endpoints', lambda: ({}, {}))
    monkeypatch.setattr(study, 'output', lambda arm: tmp_path / arm)
    (tmp_path / 'A').mkdir()
    write_a_fixture(tmp_path / 'A', state_count=400, camera_count=200)
    with pytest.raises(ValueError, match='regular non-symlink file required'):
        study.pair_receipt()
    assert not (tmp_path / 'B').exists()


def test_pair_keeps_failed_arm_and_never_promotes(monkeypatch, tmp_path):
    monkeypatch.setattr(study, 'released_endpoints', lambda: ({}, {}))
    monkeypatch.setattr(study, 'output', lambda arm: tmp_path / arm)
    for arm in ('A', 'B'):
        folder = tmp_path / arm
        folder.mkdir()
        study.write_new(folder / 'owner_plan.json', dict(schema=study.SCHEMA, arm=arm,
            binding_sha256=study.BINDING_SHA, replay_plan_sha256=study.PLAN_SHA,
            source_checkpoint_sha256=study.CHECKPOINTS[arm], source_sha256={}))
        study.write_new(folder / 'owner_diagnostic_result.json', dict(arm=arm,
            source_checkpoint_sha256=study.CHECKPOINTS[arm], status='CAPTURE_FAILED_RETAINED',
            state_count=0, camera_count=0, port_triplet_released=True, source_hashes_match=True,
            owner_cleanup_errors=[]))
    result = study.pair_receipt()
    assert result['attempt_count'] == 2 and result['two_full_20_second_captures'] is False
    assert result['model_promoted'] is False and result['learned_driving_success_claimed'] is False
    assert result['arms']['A']['status'] == 'CAPTURE_FAILED_RETAINED'
    with pytest.raises(FileExistsError):
        study.pair_receipt()


def test_cli_has_no_override_or_automatic_launch(monkeypatch):
    with pytest.raises(ValueError, match='unexpected runtime overrides'):
        study.main(['--mode', 'contract', '--max-duration-sec', '150'])
    with pytest.raises(ValueError, match='fixed replay arm required'):
        study.main(['--mode', 'execute'])
