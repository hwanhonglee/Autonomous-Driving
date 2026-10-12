"""HH_261001 - Fail closed on mixed W2 identities without selecting on future metrics."""

from copy import deepcopy
from contextlib import contextmanager
from types import SimpleNamespace
import pytest
from scripts.e2e import research_warmup_duration_w2_replay_20261001 as study


def first():
    return dict(status='PASS_EXACT_W1_STATE', expected_model_state_sha256=study.W1_STATE,
        actual_model_state_sha256=study.W1_STATE, optimizer_same_object=True, optimizer_reloaded=False,
        checkpoint_used_for_model_selection=False, checkpoint='first_block_identity_checkpoint.pt',
        checkpoint_sha256='a'*64, fit=dict(study.FIRST_FIT))


def payload():
    return dict(checkpoint_schema=study.W2_SCHEMA, arm=study.W2_ARM, parent_checkpoint_sha256=study.PARENT_SHA,
        training_plan_sha256=study.TRAIN_PLAN_SHA, training_dataset_fingerprint_sha256=study.DATASET_SHA,
        warmup_mask_sha256=study.MASK_SHA, exposure_order_sha256=study.ORDER_SHA,
        fit=deepcopy(study.FINAL_FIT), optimizer_state_dict={'state': {0: {}}}, first_block_identity=first(),
        training_data_approved=False, vehicle_control_approved=False, model_promoted=False)


def capture(arm='W1'):
    return SimpleNamespace(host='127.0.0.1',port=2440,allow_map_load=False,
        control_transport='acknowledged_batch',goal_stop_profile='disabled',physics_hz=20,capture_hz=10,
        stationary_warmup_sec=1.2,max_duration_sec=20,research_speed_cap_mps=2.5,
        target_speed_kmh=9,research_max_cte_m=1.,weather='ClearNoon',seed=0,
        output=study.output(arm)/'episode')


def test_fixed_endpoint_accounting():
    study.validate_payload(payload())
    assert study.FINAL_FIT['global_step'] == 4 * 2999
    assert study.FINAL_FIT['samples_seen'] == 4 * 11996
    assert study.FINAL_FIT['active_supervision_exposures'] == 4 * 11549


@pytest.mark.parametrize('field,value', [('global_step',11995),('scheduled_steps',11996),
    ('samples_seen',48000),('active_supervision_exposures',47984),('optimizer_resumed',True),
    ('optimizer_reset_between_blocks',True),('skipped_schedule_steps_one_based',[1693])])
def test_no_partial_or_resumed_endpoint(field,value):
    obj=payload();obj['fit'][field]=value
    with pytest.raises(ValueError):study.validate_payload(obj)


@pytest.mark.parametrize('field,value', [('status','FAIL_FIRST_BLOCK_IDENTITY'),
    ('actual_model_state_sha256','b'*64),('optimizer_same_object',False),
    ('optimizer_reloaded',True),('checkpoint_used_for_model_selection',True)])
def test_actual_first_block_proof_required(field,value):
    obj=payload();obj['first_block_identity'][field]=value
    with pytest.raises(ValueError):study.validate_payload(obj)


def test_reject_w1_schema_parent_and_missing_optimizer():
    for field,value in [('checkpoint_schema',study.W1_SCHEMA),('arm','W1_warmup_anchor_off'),
                        ('parent_checkpoint_sha256','c'*64),('optimizer_state_dict',{})]:
        obj=payload();obj[field]=value
        with pytest.raises(ValueError):study.validate_payload(obj)


@pytest.mark.parametrize('bad',['','latest','*','g'*64,'A'*64,None])
def test_no_wildcard_or_missing_identity(bad):
    with pytest.raises(ValueError):study.exact_sha(bad)


@pytest.mark.parametrize('arm,sha,schema',[('W1',study.w1.CHECKPOINT,study.W1_SCHEMA),
                                       ('W2','b'*64,study.W2_SCHEMA)])
def test_all_process_identity_restored(arm,sha,schema):
    original=study.research.AH_CHECKPOINT_SHA256,study.research.CHECKPOINT_SCHEMA
    collector=SimpleNamespace(AH_CHECKPOINT_SHA256='old')
    with pytest.raises(RuntimeError):
        with study.identity(arm,sha,collector):
            assert study.research.AH_CHECKPOINT_SHA256 == collector.AH_CHECKPOINT_SHA256 == sha
            assert study.research.CHECKPOINT_SCHEMA == schema
            raise RuntimeError('injected')
    assert (study.research.AH_CHECKPOINT_SHA256,study.research.CHECKPOINT_SCHEMA)==original
    assert collector.AH_CHECKPOINT_SHA256=='old'


def test_w1_not_substituted_for_w2():
    with pytest.raises(ValueError):
        with study.identity('W2',study.w1.CHECKPOINT):pass
    with pytest.raises(ValueError):
        with study.identity('W1','b'*64):pass


@pytest.mark.parametrize('field,value',[('max_duration_sec',21),('target_speed_kmh',30),
    ('research_speed_cap_mps',30/3.6),('research_max_cte_m',2),('goal_stop_profile','comfortable_v2'),
    ('physics_hz',10),('capture_hz',20),('stationary_warmup_sec',1.3),('allow_map_load',True)])
def test_capture_has_no_controller_or_budget_relaxation(field,value):
    args=capture();study.validate_capture(args,dict(town='Town07',scenario='straight'),'W1')
    setattr(args,field,value)
    with pytest.raises(ValueError):study.validate_capture(args,dict(town='Town07',scenario='straight'),'W1')


def test_both_arms_identical_capture_contract():
    for arm in ('W1','W2'):
        study.validate_capture(capture(arm),dict(town='Town07',scenario='straight'),arm)
    with pytest.raises(ValueError):study.validate_capture(capture(),dict(town='Town03',scenario='straight'),'W1')


def test_disk_initial_pair_and_per_arm_are_real_reserves():
    pair=study.FLOOR+2*study.ARM_RESERVE+study.SHARED_RESERVE
    assert study.disk_check(2,free=pair)['required_bytes']==pair
    with pytest.raises(ValueError):study.disk_check(2,free=pair-1)
    assert study.disk_check(1,free=pair-1)['reserved_arms']==1
    with pytest.raises(ValueError):study.disk_check(1,free=study.FLOOR)


def test_runtime_floor_restores_and_does_not_return_bundle(monkeypatch):
    calls=[]
    def original(*args,**kwargs):calls.append(True);return 'bundle'
    monkeypatch.setattr(study.short.collector,'exact_camera_bundle',original)
    monkeypatch.setattr(study.shutil,'disk_usage',lambda _:SimpleNamespace(free=study.FLOOR))
    with pytest.raises(study.short.collector.CollectionError):
        with study.disk_guard():study.short.collector.exact_camera_bundle(None)
    assert calls==[] and study.short.collector.exact_camera_bundle is original


def test_sparse_observer_is_the_exact_existing_common_implementation():
    from scripts.e2e import collect_w1_town07_long_20261001 as original
    assert study.w1.sampled_inputs is original.sampled_inputs
    assert study.w1.CHECKPOINT == 'b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76'


def test_pinned_prospective_plan_and_old_npz_unchanged():
    assert study.digest(study.PLAN)==study.PLAN_SHA
    assert study.digest(study.W1_EXPORT)==study.w1.WEIGHTS_SHA
    assert study.digest(study.W1_EXPORT.with_suffix('.json'))==study.w1.METADATA_SHA


def test_export_and_binding_are_new_exclusive_files(tmp_path):
    target=tmp_path/'receipt.json'
    study.write_new(target,{'fixed':True})
    with pytest.raises(FileExistsError):study.write_new(target,{'fixed':False})
    link=tmp_path/'link.json';link.symlink_to(target)
    with pytest.raises(ValueError):study.digest(link)


def test_bad_binding_rejected_before_loading_or_simulator(tmp_path):
    target=tmp_path/'receipt.json';study.write_new(target,{})
    with pytest.raises(ValueError):study.binding_read(target,'0'*64)
    with pytest.raises(ValueError):study.binding_read(target,study.digest(target))


def test_w1_predecessor_is_cleanup_not_progress_selected(monkeypatch,tmp_path):
    monkeypatch.setattr(study,'output',lambda _:tmp_path)
    study.write_new(tmp_path/'owner_diagnostic_result.json',dict(port_triplet_released=True,
        source_hashes_match=True,owner_cleanup_errors=[],status='CAPTURE_FAILED_RETAINED',progress_m=0))
    study.write_new(tmp_path/'owner_plan.json',dict(binding_sha256='b'*64,replay_plan_sha256=study.PLAN_SHA,
        arm='W1',source_sha256={}))
    study.prior_w1_ready('b'*64)
    with pytest.raises(ValueError):study.prior_w1_ready('c'*64)


def test_frozen_27_training_sources_unchanged():
    old=study.read_json(study.ROOT/'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/declaration.json')
    hashes=dict(old['source_sha256'])
    for name in ('tools/e2e/run_ah_warmup_duration_w2_20261001.py',
                 'learning/tests/test_warmup_duration_w2_20261001.py',
                 'docs/assets/validation/2026-10-01/selector_phase_ab/W2_LONGER_TRAINING_PREREGISTRATION.md'):
        hashes[name]=study.digest(study.ROOT/name)
    assert len(hashes)==27 and study.json_digest(hashes)==study.TRAIN_SOURCE_SHA
    assert all(study.digest(study.ROOT/name)==sha for name,sha in hashes.items())


def test_owner_preflight_command_no_launch_and_bindings_restore(monkeypatch,tmp_path):
    from scripts.e2e import run_owned_ah_town07_research_20261001 as owner
    saved=(owner.OUTPUT,owner.WEIGHTS,owner.METADATA,owner.PINS,owner.collector_command,
           owner.write_json,owner.lifecycle._workspace_lock)
    monkeypatch.setattr(study,'binding_read',lambda *_:{})
    monkeypatch.setattr(study,'output',lambda arm:tmp_path/arm)
    monkeypatch.setattr(study,'disk_check',lambda arms=1:dict(reserved_arms=arms))
    monkeypatch.setattr(study,'load_endpoint',lambda *_:None)
    monkeypatch.setattr(owner,'preflight',lambda:dict(collector_command=owner.collector_command()))
    def forbidden(*args,**kwargs):raise AssertionError('preflight launched simulator')
    monkeypatch.setattr(owner,'execute',forbidden)
    args=SimpleNamespace(binding=tmp_path/'binding.json',binding_sha256='a'*64,arm='W1',mode='preflight')
    result=study.trial(args)
    command=result['collector_command']
    assert 'scripts.e2e.research_warmup_duration_w2_replay_20261001' in command
    assert command[command.index('--target-speed-kmh')+1]=='9'
    assert command[command.index('--max-duration-sec')+1]=='20'
    assert command[command.index('--binding-sha256')+1]=='a'*64
    assert '--research-weights' not in command
    assert result['source_checkpoint_sha256']==study.w1.CHECKPOINT
    assert result['source_checkpoint_schema']==study.W1_SCHEMA
    assert result['disk_reservation']['reserved_arms']==2
    assert (owner.OUTPUT,owner.WEIGHTS,owner.METADATA,owner.PINS,owner.collector_command,
            owner.write_json,owner.lifecycle._workspace_lock)==saved


def test_real_w1_npz_recomputes_exact_tensor_identity():
    import torch
    torch.set_num_threads(2)
    item=study.endpoint('W1',{})
    model=study.load_endpoint('W1',item)
    assert study.tensor_digest(model.state_dict())==study.W1_STATE
    assert model._research_source_checkpoint_sha256==study.w1.CHECKPOINT


def test_declaration_does_not_select_on_final_metrics():
    import inspect
    source=inspect.getsource(study.export)
    assert "summary.get('status') == 'COMPLETE_NOT_PROMOTED'" in source
    for forbidden in ("summary['strict_gates']", "summary['evaluation']", "train_probe_after", 'final not in'):
        assert forbidden not in source
