"""HH_261001 - Guard new-arm semantics, fixed panels, photo trace and historical preservation."""

import copy
import numpy as np
import pytest
from scripts.e2e import evaluate_map_photometric_endpoints_20261001 as evaluate
from scripts.e2e import verify_map_photometric_endpoints_20261001 as verify


def fixture():
    study,registry,manifest = evaluate.package_binding()
    common = {'study_id':evaluate.photo.NAME,'data_manifest_sha256':evaluate.DATA_SHA,
        'source_bundle_sha256':evaluate.SOURCE_SHA,'study_registry_sha256':evaluate.REGISTRY_SHA,
        'parent_weights_sha256':evaluate.contract.PARENT_SHA,'steps_per_arm':10000,'batch':8,
        'common_class_weights':manifest['common_class_weights'],'schedule_contract':manifest['schedule_contract'],
        'reference_data_manifest_sha256':evaluate.photo.REFERENCE_MANIFEST_SHA,
        'photometric_policy':evaluate.photo.policy(),'expected_photo_trace':study['expected_photo_trace'],
        'fresh_fits':2,'ctrack_train_eligible':False,'model_promotion':False}
    info = {'steps':10000,'parameter_count':245831,'initial_state_sha256':evaluate.previous.INITIAL_STATE_SHA,
        'optimizer_initial_state_entries':0,'sample_sequence_sha256':manifest['schedule_contract']['indices_sha256'],
        'crop_sequence_sha256':manifest['schedule_contract']['crops_sha256']}
    arms = {'control':{**info,'photo_trace':evaluate.photo.Trace().receipt(np.random.default_rng(evaluate.photo.PHOTO_SEED)),
        'predecessor_reference_state_reproduced':True,'final_state_sha256':study['predecessor_appearance_state_sha256']},
        'photometric':{**info,'photo_trace':study['expected_photo_trace'],'predecessor_reference_state_reproduced':None}}
    summary = {**common,'status':'PHOTOMETRIC_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED','arms':arms,
        'dev_used_during_optimization':False,'source_and_data_unchanged':True}
    complete = {'status':'BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV','arms':arms}
    return summary,common,complete,study,manifest


def test_policy_and_historical_module_unchanged():
    policy = evaluate.policy()
    assert policy['arms'] == ['control','photometric']
    assert evaluate.previous.ARMS == ('control','appearance')
    assert policy['scenarios'] == ['ctrack_left','ctrack_straight','train_town01_p01','train_town02_p00']
    assert policy['visual_indices'] == [0,119,239] and policy['frames_per_scenario'] == 240
    assert 'TRUE480' in policy['control'] and 'never previous FALSE-control' in policy['primary_comparison']
    assert not policy['ctrack_train_eligible'] and policy['performance_gates'] is None
    assert policy['openblas_threads'] == 2 and not policy['inference_augmentation']
    assert evaluate.output_path('ctrack_left') != evaluate.previous.output_path('ctrack_left')


def test_accepts_only_fixed_same_data_photo_pair():
    evaluate.validate_endpoint_metadata(*fixture())


@pytest.mark.parametrize('field,value',[('steps',9999),('parameter_count',130503),('initial_state_sha256','other'),
    ('sample_sequence_sha256','other'),('crop_sequence_sha256','other'),('optimizer_initial_state_entries',1)])
def test_rejects_endpoint_interventions(field,value):
    arguments = copy.deepcopy(fixture())
    arguments[2]['arms']['photometric'][field] = value
    with pytest.raises(ValueError):
        evaluate.validate_endpoint_metadata(*arguments)


@pytest.mark.parametrize('arm,field,value',[('control','predecessor_reference_state_reproduced',False),
    ('control','final_state_sha256','not_previous_appearance'),('control','photo_trace',{}),
    ('photometric','photo_trace',{}),('photometric','predecessor_reference_state_reproduced',True)])
def test_rejects_false_control_or_changed_augmentation(arm,field,value):
    arguments = copy.deepcopy(fixture()); arguments[2]['arms'][arm][field] = value
    with pytest.raises(ValueError):
        evaluate.validate_endpoint_metadata(*arguments)


@pytest.mark.parametrize('conditions',[('true',),('false','true')])
def test_every_native_panel_pixel_is_preserved(conditions):
    rng = np.random.default_rng(261001)
    rgb = {condition:rng.integers(0,256,(360,640,3),np.uint8) for condition in conditions}
    target = rng.integers(0,7,(360,640),np.uint8)
    predictions = {arm+'_'+condition:rng.integers(0,7,(360,640),np.uint8) for condition in conditions for arm in evaluate.ARMS}
    image = np.asarray(evaluate.render('test',119,rgb,target,predictions,conditions))
    assert image.shape == (1640 if len(conditions)==2 else 860,1280,3)
    for x,y,pixels in evaluate.panel_layout(rgb,target,predictions,conditions):
        assert np.array_equal(image[y:y+360,x:x+640],pixels)


def test_independent_absent_class_keeps_false_positives():
    matrix = verify.independent_confusion(np.zeros((2,2),np.uint8), np.ones((2,2),np.uint8))
    result = evaluate.frozen.metrics(matrix); verify.check_metrics(matrix,result)
    assert result['class_metrics']['road']['iou'] is None
    assert result['class_metrics']['road']['false_positive_pixels'] == 4
