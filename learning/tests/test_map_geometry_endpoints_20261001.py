"""HH_261001 - Fixed geometry diagnostics remain paired, complete and storage bounded."""

import copy
import hashlib
import io
import json
from pathlib import Path
import numpy as np
import pytest
from scripts.e2e import evaluate_map_geometry_endpoints_20261001 as evaluate
from scripts.e2e import verify_map_geometry_endpoints_20261001 as verify


def fixture():
    _, plan = evaluate.package_metadata()
    common = dict(study_id=evaluate.geometry.NAME, source_bundle_sha256=evaluate.SOURCE_SHA,
        prepared_manifest_sha256=evaluate.geometry.DATA_SHA, reference_plan_sha256=evaluate.geometry.PLAN_SHA,
        package_authorization_sha256=evaluate.geometry.PACKAGE_APPROVAL_SHA,
        fit_authorization_sha256=evaluate.FIT_APPROVAL_SHA,
        parent_weights_sha256=evaluate.contract.PARENT_SHA, common_class_weights=evaluate.geometry.CLASS_WEIGHTS,
        schedule=plan['schedule'], photometric=plan['photometric'], fresh_fits=2, steps_per_arm=10000,
        batch=8, learning_rate=evaluate.contract.LR, weight_decay=evaluate.contract.WEIGHT_DECAY,
        gradient_clip_norm=5., same_architecture=True, ctrack_train_eligible=False,
        broad_registry_modified=False, model_promotion=False)
    indices, _ = evaluate.geometry.package.schedule()
    replay = evaluate.geometry.package.control_replay()
    arms = {arm:dict(complete=True, steps=10000, parameter_count=245831, optimizer_initial_state_entries=0,
        initial_state_sha256=evaluate.historical.previous.INITIAL_STATE_SHA,
        sample_sequence_sha256=plan['schedule']['indices_int64_sha256'],
        crop_sequence_sha256=plan['schedule']['crops_int16_sha256'], common_class_weights=evaluate.geometry.CLASS_WEIGHTS,
        photo_trace={k:plan['photometric'][k] for k in ('eligible_calls','applied_calls','sample_gate_trace_sha256','final_rng_state')},
        exposure=evaluate.geometry.exposure_receipt(indices,replay,arm)) for arm in evaluate.ARMS}
    return ({**common,'status':'GEOMETRY_SOURCE_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED','arms':arms,
        'dev_used_during_optimization':False,'source_and_data_unchanged':True}, common,
        dict(status='BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV',arms=arms), plan)


def test_fixed_protocol_and_historical_sources_untouched():
    policy = evaluate.policy()
    assert policy['arms'] == ['control','geometry']
    assert evaluate.historical.ARMS == ('control','photometric')
    assert policy['scenarios'] == ['ctrack_left','ctrack_straight','train_town01_p01','train_town02_p00']
    assert policy['visual_indices'] == [0,119,239] and policy['frames_per_scenario'] == 240
    assert not policy['ctrack_train_eligible'] and policy['performance_gates'] is None
    assert policy['maximum_output_bytes'] == 30*1024**2
    assert evaluate.output_path('ctrack_left') != evaluate.historical.output_path('ctrack_left')
    assert 'different exposure' in policy['historical_comparison']


def test_fixed_endpoint_contract():
    evaluate.validate_endpoint_metadata(*fixture())


@pytest.mark.parametrize('key,value',[('complete',False),('steps',9999),('parameter_count',130503),
    ('optimizer_initial_state_entries',1),('initial_state_sha256','wrong'),('sample_sequence_sha256','wrong'),
    ('crop_sequence_sha256','wrong'),('photo_trace',{}),('exposure',{}),('common_class_weights',[1.]*7)])
def test_rejects_endpoint_changes(key,value):
    args = copy.deepcopy(fixture()); args[2]['arms']['geometry'][key] = value
    with pytest.raises(ValueError):
        evaluate.validate_endpoint_metadata(*args)


def test_rejects_one_endpoint_and_dev_tuning():
    args = copy.deepcopy(fixture()); args[0]['dev_used_during_optimization'] = True
    with pytest.raises(ValueError): evaluate.validate_endpoint_metadata(*args)
    args = copy.deepcopy(fixture()); del args[2]['arms']['geometry']
    with pytest.raises(ValueError): evaluate.validate_endpoint_metadata(*args)


@pytest.mark.parametrize('conditions',[('true',),('false','true')])
def test_native_rgb_gt_and_both_new_predictions_unscaled(conditions):
    rng = np.random.default_rng(261001)
    rgb = {k:rng.integers(0,256,(360,640,3),np.uint8) for k in conditions}
    target = rng.integers(0,7,(360,640),np.uint8)
    pred = {arm+'_'+k:rng.integers(0,7,(360,640),np.uint8) for k in conditions for arm in evaluate.ARMS}
    canvas = np.asarray(evaluate.render('synthetic',119,rgb,target,pred,conditions))
    assert canvas.shape == (1640 if len(conditions)==2 else 860,1280,3)
    for x,y,pixels in evaluate.panel_layout(rgb,target,pred,conditions):
        assert np.array_equal(canvas[y:y+360,x:x+640],pixels)


def test_budget_rejects_before_writing_preserves_prior_bytes(tmp_path,monkeypatch):
    root = tmp_path/'results'; scenario = root/'synthetic'; scenario.mkdir(parents=True)
    monkeypatch.setattr(evaluate,'OUTPUT',root); monkeypatch.setattr(evaluate,'MAX_OUTPUT_BYTES',100)
    monkeypatch.setattr(evaluate,'FAILURE_RESERVE_BYTES',20); monkeypatch.setattr(evaluate,'FREE_FLOOR_BYTES',0)
    evaluate.write_artifact(scenario/'one',b'a'*40)
    with pytest.raises(ValueError,match='BUDGET'):
        evaluate.write_artifact(scenario/'too_large',b'b'*40)
    assert not (scenario/'too_large').exists() and (scenario/'one').read_bytes() == b'a'*40
    evaluate.write_artifact(scenario/'failure.json',b'failure',failure=True)
    assert evaluate.used_bytes() == 47


def test_budget_path_symlink_and_overwrite_rejected(tmp_path,monkeypatch):
    root = tmp_path/'results'; scenario = root/'synthetic'; scenario.mkdir(parents=True)
    monkeypatch.setattr(evaluate,'OUTPUT',root); monkeypatch.setattr(evaluate,'FREE_FLOOR_BYTES',0)
    evaluate.write_artifact(scenario/'one',b'a')
    with pytest.raises(FileExistsError): evaluate.write_artifact(scenario/'one',b'b')
    with pytest.raises(ValueError): evaluate.write_artifact(tmp_path/'outside',b'b')
    (scenario/'alias').symlink_to(scenario/'one')
    with pytest.raises(ValueError,match='symlink'): evaluate.used_bytes()


def test_numeric_receipt_float32_no_pickle(tmp_path):
    value = np.arange(6,dtype=np.float32).reshape(2,3)
    path = tmp_path/'safe.npz'; np.savez_compressed(path,weight=value)
    header = json.dumps(['weight','torch.float32',[2,3]]).encode()
    sha = hashlib.sha256(len(header).to_bytes(8,'big')+header+value.tobytes()).hexdigest()
    info = dict(final_state_sha256=sha,final_tensors={'weight':dict(shape=[2,3],dtype='float32',sha256=hashlib.sha256(value.tobytes()).hexdigest())})
    evaluate.verify_numeric_npz(path,info)
    with pytest.raises(ValueError): evaluate.verify_numeric_npz(path,{**info,'final_state_sha256':'wrong'})
    np.savez_compressed(tmp_path/'wrong.npz',weight=value.astype(np.float64))
    with pytest.raises(ValueError): evaluate.verify_numeric_npz(tmp_path/'wrong.npz',info)


def test_recount_preserves_absent_class_false_positives():
    matrix = verify.independent_confusion(np.zeros((2,2),np.uint8),np.ones((2,2),np.uint8))
    result = evaluate.frozen.metrics(matrix); verify.check_metrics(matrix,result)
    assert result['class_metrics']['road']['iou'] is None
    assert result['class_metrics']['road']['false_positive_pixels'] == 4


def test_known350_cannot_be_merged_or_count_changed():
    value = {}
    for split,count in (('dev_town01',118),('holdout_town02_p02',232)):
        matrix = np.zeros((7,7),np.int64); matrix[0,0] = count*360*640
        value[split] = {arm:dict(count=count,confusion=matrix.tolist()) for arm in evaluate.ARMS}
    evaluate.validate_known_receipt(value)
    value['dev_town01']['control']['count'] = 350
    with pytest.raises(ValueError): evaluate.validate_known_receipt(value)
