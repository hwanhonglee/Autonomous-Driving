"""HH_261001 - Deny local arrays, missing authority, and augmentation leakage to new FALSE slots."""
import argparse
import hashlib
import numpy as np
import pytest
from scripts.e2e import prepare_map_route_geometry_remote_20261001 as package


def test_fixed_same_town_replay():
    ids = package.control_replay()
    assert ids.shape == (480,) and ids.dtype == np.int64
    assert len(set(ids.tolist())) == 480 and ids.min() >= 0 and ids.max() < 497
    assert np.array_equal(ids,np.random.default_rng(20261002).permutation(497)[:480])


def test_new_false_not_photo_augmented():
    assert not package.eligible_for_photo(728)
    assert package.eligible_for_photo(729) and package.eligible_for_photo(1208)
    assert not package.eligible_for_photo(1209) and not package.eligible_for_photo(1688)


def test_authority_is_not_inferred():
    with pytest.raises(ValueError,match='authorization'):
        package.validate_authorization({},'0'*64)


def test_materialization_denies_local_host():
    with pytest.raises(ValueError,match='personal remote host'):
        package.materialize(argparse.Namespace())


def test_matched_exposure_totals():
    value = package.plan()
    schedule = value['schedule']
    assert sum(schedule[k] for k in ('original_false_exposures','paired_true_exposures','extra_false_exposures')) == 80000
    assert schedule['paired_true_exposures'] == value['photometric']['eligible_calls']
    assert schedule['crops_int16_sha256'] == '19077c804ffc32131ea1851388ae943bdf848e206fb2cc9e7de691e9866e05ba'
    assert schedule['full_epochs']*1689+schedule['last_partial_epoch'] == 80000


def test_aug_trace_matches_actual_archived_policy():
    from scripts.e2e import train_map_photometric_pilot as archived
    assert package.sha(package.LOCAL_MANIFEST.parents[6]/'tools/e2e/train_map_photometric_pilot.py') == package.POLICY_SHA
    ids,_ = package.schedule()
    rng = np.random.default_rng(20261003)
    digest = hashlib.sha256()
    applied = 0
    for index in ids.ravel():
        if package.eligible_for_photo(index):
            _,active = archived.photometric(np.zeros((1,1,3),np.uint8),rng)
            digest.update(np.array([index,int(active)],np.int64).tobytes())
            applied += int(active)
    expected = package.plan()['photometric']
    assert digest.hexdigest() == expected['sample_gate_trace_sha256']
    assert applied == expected['applied_calls']
    assert rng.bit_generator.state == expected['final_rng_state']


def test_transfer_references_only_candidate_arrays():
    files = package.transfer_files(package.LOCAL_MANIFEST)
    arrays = [f for f in files if f.endswith('.npz')]
    assert len(files) == 1008 and len(arrays) == 480
    assert all('/map_sign_native_p02_probe/' in f or '/map_sign_native_p04_probe/' in f for f in arrays)
