"""HH_260906 - Check conservative interval proofs using synthetic metadata, never opening a real dataset."""
import copy
from itertools import product
import pytest
from scripts.e2e import audit_nuplan_camera_subset_intervals as audit

def fixture():
    values = {'CAM_B0': 25000, 'CAM_F0': 0, 'CAM_L0': -7700, 'CAM_L1': 34700,
              'CAM_L2': 30300, 'CAM_R0': 8100, 'CAM_R1': 15000, 'CAM_R2': 19800}
    return {'schema': 'nuplan.single_db_metadata_probe.v1', 'status': 'METADATA_INSPECTED_NOT_READY',
        'metadata': {'camera_alignment': {'reference_channel': 'CAM_F0', 'reference_anchor_count': 3760,
        'per_channel_signed_offset_us': {c: {'offset': {'count': 3760,
        'minimum': v if c == 'CAM_F0' else v - 50, 'maximum': v if c == 'CAM_F0' else v + 50}} for c, v in values.items()}}}}

def test_all_28_subsets_not_a_rig_selection():
    value = fixture(); before = copy.deepcopy(value)
    result = audit.analyze(value)
    assert value == before and result['subset_count'] == 28
    assert sum(row['includes_front_reference_camera'] for row in result['subsets']) == 21
    assert result['minimum_six_subset_lower_bound_us'] == 26500
    assert result['minimum_front_including_subset_lower_bound_us'] == 30250
    assert result['all_nearest_front_six_subsets_exceed_20ms'] is True
    assert all(result[k] is False for k in ('rig_selected', 'training_data_approved', 'timestamp_modified', 'new_db_or_camera_payload_read'))

@pytest.mark.parametrize('intervals', [((0, 2),) * 6, ((-2, 0), (4, 7), (1, 5), (0, 10), (2, 8), (3, 6)),
    ((-10, -9), (-8, -7), (-3, -1), (2, 3), (5, 7), (8, 11))])
def test_bound_is_no_larger_than_every_endpoint_assignment(intervals):
    bound = audit.interval_lower_bound(intervals)
    assert all(max(assignment) - min(assignment) >= bound for assignment in product(*intervals))

@pytest.mark.parametrize('bad', [[], [[0, 1]] * 5, [[1, 0]] * 6, [[False, 1]] * 6, [[0.0, 1]] * 6])
def test_invalid_intervals_fail_closed(bad):
    with pytest.raises(ValueError): audit.interval_lower_bound(bad)

def test_overlapping_intervals_do_not_claim_a_failure():
    value = fixture()
    for c, row in value['metadata']['camera_alignment']['per_channel_signed_offset_us'].items():
        if c != 'CAM_F0': row['offset'].update(minimum=-100, maximum=100)
    result = audit.analyze(value)
    assert result['minimum_six_subset_lower_bound_us'] == 0
    assert result['all_nearest_front_six_subsets_exceed_20ms'] is False

@pytest.mark.parametrize('key,value', [('reference_channel', 'CAM_B0'), ('reference_anchor_count', 1)])
def test_reference_contract_cannot_drift(key, value):
    report = fixture(); report['metadata']['camera_alignment'][key] = value
    with pytest.raises(ValueError): audit.analyze(report)

def test_incomplete_channel_statistics_are_rejected():
    report = fixture(); report['metadata']['camera_alignment']['per_channel_signed_offset_us']['CAM_B0']['offset']['count'] = 3759
    with pytest.raises(ValueError): audit.analyze(report)
