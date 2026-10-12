"""HH_261001 - Freeze the source-only factor, admission guards, endpoint ordering and real parent ABI."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest

from scripts.e2e import map_route_geometry_contract_20261001 as c
from scripts.e2e import train_map_route_geometry_matched_20261001 as train

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT/'docs/assets/validation/2026-10-01/map_unused_route_inventory'
BASE = ROOT/'storage/experiments/training/2026-10-01/map_appearance_matched_20261001_v1'


def read(path):
    return json.loads(path.read_text())


def test_actual_remote_manifest_contract():
    assert c.base.digest(DOC/'remote_package_manifest.json') == c.DATA_SHA
    c.validate_prepared_manifest(read(DOC/'remote_package_manifest.json'),read(BASE/'manifest.json'))
    assert c.base.digest(DOC/'package_authorization.json') == c.PACKAGE_APPROVAL_SHA


@pytest.mark.parametrize('mutation',['drop','reorder','wrong_weights','alter_file','wrong_parent','fit_flag'])
def test_prepared_mutations_rejected(mutation):
    value = read(DOC/'remote_package_manifest.json')
    if mutation == 'drop':
        value['source_ids'].pop()
    elif mutation == 'reorder':
        value['source_ids'] = value['source_ids'][::-1]
    elif mutation == 'wrong_weights':
        value['common_class_weights'][3] += .001
    elif mutation == 'alter_file':
        value['files']['new_false480.npz'] = '0'*64
    elif mutation == 'wrong_parent':
        value['parent_weights_sha256'] = '0'*64
    else:
        value['fit_authorized'] = True
    with pytest.raises(ValueError):
        c.validate_prepared_manifest(value,read(BASE/'manifest.json'))


def test_full_schedule_exact_and_no_protected_sources():
    plan = read(DOC/'reference_plan.json')
    assert c.base.digest(DOC/'reference_plan.json') == c.PLAN_SHA
    c.validate_plan(plan)
    replay = c.package.control_replay()
    for arm in c.ARMS:
        assert c.slot_source(0,arm,replay) == ('original',0)
        assert c.slot_source(728,arm,replay) == ('original',728)
        assert c.slot_source(729,arm,replay) == ('paired',0)
        assert c.slot_source(1208,arm,replay) == ('paired',479)
    assert c.slot_source(1209,'control',replay) == ('original',int(replay[0]))
    assert c.slot_source(1209,'geometry',replay) == ('geometry',0)
    assert c.slot_source(1449,'geometry',replay) == ('geometry',240)
    assert c.slot_source(1688,'geometry',replay) == ('geometry',479)
    with pytest.raises(ValueError):
        c.slot_source(1689,'geometry',replay)


def test_actual_physical_exposure_different_only_extra_sources():
    ids,_ = c.package.schedule()
    replay = c.package.control_replay()
    a,b = [c.exposure_receipt(ids,replay,arm) for arm in c.ARMS]
    assert a['slot_exposures'] == b['slot_exposures']
    assert a['physical_source_exposures']['paired'] == b['physical_source_exposures']['paired']
    assert a['physical_source_exposures']['original'][497:] == b['physical_source_exposures']['original'][497:]
    assert a['physical_source_totals'] == {'original':57257,'paired':22743,'geometry':0}
    assert b['physical_source_totals'] == {'original':34507,'paired':22743,'geometry':22750}


def test_both_arms_same_photo_rng_and_no_false_changes():
    from scripts.e2e.train_map_photometric_pilot import photometric
    ids = np.array([0,729,1208,1209,1688])
    inputs = np.arange(60,dtype=np.uint8).reshape(5,2,2,3)
    results = []
    for _ in c.ARMS:
        rng,trace = np.random.default_rng(20261003),c.PhotoTrace()
        output = c.augment_both_arms(inputs,ids,rng,photometric,trace)
        assert np.array_equal(output[[0,3,4]],inputs[[0,3,4]])
        assert trace.eligible == 2
        results.append((output,trace.receipt(rng)))
    assert np.array_equal(results[0][0],results[1][0]) and results[0][1] == results[1][1]


def test_actual_full_photo_trace_matches_preregistered():
    from scripts.e2e.train_map_photometric_pilot import photometric
    ids,_ = c.package.schedule()
    rng,trace = np.random.default_rng(20261003),c.PhotoTrace()
    for batch in ids:
        c.augment_both_arms(np.zeros((8,1,1,3),np.uint8),batch,rng,photometric,trace)
    expected = read(DOC/'reference_plan.json')['photometric']
    assert trace.receipt(rng) == {k:expected[k] for k in trace.receipt(rng)}


def test_package_authority_cannot_authorize_fit():
    with pytest.raises(ValueError,match='fit authorization'):
        c.validate_fit_authorization(read(DOC/'package_authorization.json'),c.DATA_SHA,'a'*64)


def test_exact_fit_authorization_requires_pinned_weights_and_hashes():
    value = {'schema':'portable_e2e.map_geometry_fit_authorization.v1','study_id':c.NAME,
        'fit_authorized':True,'prepared_manifest_sha256':c.DATA_SHA,'source_bundle_sha256':'a'*64,
        'reference_plan_sha256':c.PLAN_SHA,'package_authorization_sha256':c.PACKAGE_APPROVAL_SHA,
        'steps_per_arm':10000,'arms':list(c.ARMS),'common_class_weights':list(c.CLASS_WEIGHTS),
        'no_dev_checkpoint_selection':True,'ctrack_train_eligible':False}
    c.validate_fit_authorization(value,c.DATA_SHA,'a'*64)
    value['common_class_weights'][0] = 1.
    with pytest.raises(ValueError):
        c.validate_fit_authorization(value,c.DATA_SHA,'a'*64)


def test_dev_guard_requires_two_complete_immutable_finals(tmp_path):
    with pytest.raises(ValueError,match='Both fixed'):
        train.validate_endpoint_pair({},tmp_path)
    arms = {}
    for name in c.ARMS:
        root = tmp_path/name
        root.mkdir()
        (root/'weights.npz').write_bytes(b'weights')
        (root/'checkpoint.pt').write_bytes(b'checkpoint')
        arms[name] = {'steps':10000,'complete':True,'weights_sha256':c.base.digest(root/'weights.npz'),
                      'checkpoint_sha256':c.base.digest(root/'checkpoint.pt')}
    train.validate_endpoint_pair(arms,tmp_path)
    arms['geometry']['steps'] = 9999
    with pytest.raises(ValueError,match='DEV must stay unopened'):
        train.validate_endpoint_pair(arms,tmp_path)


def test_absent_class_null_retains_false_positives():
    confusion = np.zeros((7,7),np.int64)
    confusion[1,1],confusion[1,5] = 8,2
    m = train.metrics(confusion)
    assert m['iou']['traffic_sign'] is None and m['false_positive_pixels']['traffic_sign'] == 2
    assert m['iou']['road'] == .8


def test_local_training_denied_and_budgets_below_45_minutes():
    with pytest.raises(ValueError,match='personal host'):
        train.environment_contract(False)
    assert c.FIT_SECONDS+c.EVALUATION_SECONDS < 45*60
    with pytest.raises(TimeoutError):
        train.deadline_check(0)


def test_real_parent_safe_npz_tensor_abi_cpu_only(tmp_path):
    from portable_e2e.map_context_recovery import MapContextRecovery
    from scripts.e2e.train_map_photometric_pilot import state_digest
    model = MapContextRecovery()
    train.load_weights(model,BASE/'parent_weights.npz',c.base.PARENT_SHA)
    assert sum(p.numel() for p in model.parameters()) == 245831
    assert state_digest(model) == 'edf12b02e3bf34526ce30de6fb450c2fa91085004e5dbd42c95c9e159d6a2b67'
    receipt = train.save_tensor_npz(model,tmp_path/'weights.npz')
    assert set(receipt) == set(model.state_dict())
    assert all(row['dtype'] == 'float32' for row in receipt.values())
