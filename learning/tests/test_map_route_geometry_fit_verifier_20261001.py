"""HH_261001 - Exercise independent audit math and rejection paths without inspecting active fit outputs."""

from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from scripts.e2e import verify_map_route_geometry_fit_20261001 as verify


def test_exact_frozen_source_and_input_pins_before_results():
    root = verify.DOC/'training_source_v1'
    assert verify.digest(root/'bundle.json') == verify.SOURCE_SHA
    bundle = verify.read_json(root/'bundle.json')
    assert bundle['files'] == verify.SOURCE_FILES and len(verify.SOURCE_FILES) == 9
    for name,expected in verify.SOURCE_FILES.items():
        assert verify.digest(root/name) == expected
    assert verify.digest(verify.DOC/'remote_package_manifest.json') == verify.DATA_SHA
    assert verify.digest(verify.DOC/'fit_authorization.json') == verify.FIT_AUTH_SHA


def test_independent_schedule_matches_immutable_pre_fit_plan():
    actual = verify.independent_schedule()
    plan = verify.read_json(verify.DOC/'reference_plan.json')
    assert actual['sample_sha256'] == plan['schedule']['indices_int64_sha256']
    assert actual['crop_sha256'] == plan['schedule']['crops_int16_sha256']
    assert actual['replay'].tolist() == plan['control_original_indices']
    assert actual['photo_trace'] == {k:plan['photometric'][k] for k in actual['photo_trace']}
    assert len(actual['flat']) == 80000


def test_original_and_true_retention_independently_reconstructed():
    algorithm = verify.independent_schedule()
    control,geometry = [verify.independent_exposure(algorithm['flat'],algorithm['replay'],arm) for arm in verify.ARMS]
    assert control['slot_exposures'] == geometry['slot_exposures']
    assert control['physical_source_exposures']['paired'] == geometry['physical_source_exposures']['paired']
    assert control['physical_source_exposures']['original'][497:] == geometry['physical_source_exposures']['original'][497:]
    assert min(geometry['physical_source_exposures']['original']) == 47
    assert min(geometry['physical_source_exposures']['geometry']) == 47
    assert control['physical_source_totals'] == {'original':57257,'paired':22743,'geometry':0}
    assert geometry['physical_source_totals'] == {'original':34507,'paired':22743,'geometry':22750}


def history_fixture():
    return [{'arm':'control','step':i*100,'mean_loss_100':.1,'wall_seconds':i*.1} for i in range(1,101)]


def test_full_history_accepted():
    value = verify.check_history(history_fixture(),'control',{'steps':10000,'complete':True,'training_wall_seconds':11.})
    assert value['recorded_intervals'] == 100 and value['final_recorded_step'] == 10000


@pytest.mark.parametrize('mutation',['missing','reordered','nonfinite_loss','wrong_arm','nonmonotonic_time','wrong_step'])
def test_invalid_histories_rejected(mutation):
    rows = history_fixture()
    if mutation == 'missing':
        rows.pop()
    elif mutation == 'reordered':
        rows[0],rows[1] = rows[1],rows[0]
    elif mutation == 'nonfinite_loss':
        rows[5]['mean_loss_100'] = float('nan')
    elif mutation == 'wrong_arm':
        rows[5]['arm'] = 'geometry'
    elif mutation == 'nonmonotonic_time':
        rows[5]['wall_seconds'] = rows[4]['wall_seconds']
    else:
        rows[-1]['step'] = 9999
    with pytest.raises(ValueError):
        verify.check_history(rows,'control',{'steps':10000,'complete':True,'training_wall_seconds':11.})


def test_actual_parent_state_without_torch_or_pickle():
    assert verify.digest(verify.PARENT) == verify.PARENT_SHA
    receipt,state,count = verify.tensor_archive(verify.PARENT)
    assert state == verify.INITIAL_SHA and count == 245831
    assert all(row['dtype'] == 'float32' for row in receipt.values())


@pytest.mark.parametrize('mutation',['shape','dtype','nan','keys'])
def test_unsafe_or_wrong_tensor_abi_rejected(tmp_path,mutation):
    expected = {'x':{'shape':[2,3]}}
    value = np.zeros((2,3),np.float32)
    key = 'x'
    if mutation == 'shape':
        value = value.reshape(3,2)
    elif mutation == 'dtype':
        value = value.astype(np.float64)
    elif mutation == 'nan':
        value[0,0] = float('nan')
    else:
        key = 'y'
    path = tmp_path/'weights.npz'
    np.savez_compressed(path,**{key:value})
    with pytest.raises(ValueError):
        verify.tensor_archive(path,expected)


def test_independent_all_pixel_count_and_absent_false_positive():
    target = np.array([[[1,1],[2,2]],[[1,3],[3,0]]],np.uint8)
    pred = np.array([[[1,5],[2,2]],[[2,3],[0,0]]],np.uint8)
    result = verify.recount(target,pred)
    assert result.sum() == 8 and result[1,5] == 1 and result[1,2] == 1
    metrics = verify.independent_metrics(result)
    assert metrics['iou']['traffic_sign'] is None
    assert metrics['false_positive_pixels']['traffic_sign'] == 1
    assert metrics['target_pixels']['road'] == 3
    assert metrics['iou']['road'] == 1/3


@pytest.mark.parametrize('mutation',['class','shape','dtype'])
def test_invalid_saved_predictions_rejected(mutation):
    target = np.zeros((2,3,4),np.uint8)
    pred = target.copy()
    if mutation == 'class':
        pred[0,0,0] = 7
    elif mutation == 'shape':
        pred = pred[:1]
    else:
        pred = pred.astype(np.int16)
    with pytest.raises(ValueError):
        verify.recount(target,pred)


def test_summary_first_refuses_incomplete_run(tmp_path,monkeypatch):
    monkeypatch.setattr(verify,'FIT',tmp_path)
    path = tmp_path/'summary.json'
    path.write_text(json.dumps({'status':'RUNNING'}))
    with pytest.raises(ValueError,match='Completed diagnostic pair'):
        verify.verify(verify.digest(path))


def test_failure_receipt_refuses_even_before_summary(tmp_path,monkeypatch):
    monkeypatch.setattr(verify,'FIT',tmp_path)
    (tmp_path/'failure.json').write_text('{}')
    with pytest.raises(ValueError,match='Failed run'):
        verify.verify('0'*64)


def test_nonfinite_receipt_json_rejected(tmp_path):
    p = tmp_path/'bad.json'
    p.write_text('{"metric": NaN}')
    with pytest.raises(ValueError,match='Nonfinite JSON'):
        verify.read_json(p)
