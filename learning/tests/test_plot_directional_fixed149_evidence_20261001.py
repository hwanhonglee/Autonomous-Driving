"""HH_261001 - Frozen record plots fail closed on frame mismatch, changed arrays and clipped coordinates."""

from copy import deepcopy
import inspect
import pytest
from scripts.e2e import plot_directional_fixed149_evidence_20261001 as plot


@pytest.fixture(scope='module')
def actual():
    return plot.load()


def test_all149_values_and_cases_exact(actual):
    records, data, associations, sources = actual
    assert data['overview']['sequence'] == list(range(149))
    assert sum(data['overview']['A']['hold_rule_proposal']) == 40
    assert sum(data['overview']['B']['hold_rule_proposal']) == 8
    for seq in plot.CASES:
        assert data['cases'][str(seq)]['time_s'] == [(i+1)*.1 for i in range(64)]
        for arm in ('A','B'):
            assert data['cases'][str(seq)][arm]['xy_m'] == records[seq][arm]['selected_xy_m']
            assert data['cases'][str(seq)][arm]['speed_mps'] == records[seq][arm]['selected_speed_mps']
    assert associations['3']['frame'] == 1400 and associations['117']['frame'] == 3680
    assert len(sources) == 14


def test_missing_or_duplicate_frame_never_guessed(actual):
    records = actual[0]
    state = plot.rows(plot.EPISODE/'states.jsonl')
    camera = plot.rows(plot.EPISODE/'camera_frames.jsonl')
    index = plot.rows(plot.TRIAL/'sampled_exact_runtime_inputs/index.jsonl')
    controls = plot.rows(plot.EPISODE/'learned_control.jsonl')
    for wrong in ([r for r in camera if r['frame'] != 1400], camera+[r for r in camera if r['frame']==1400]):
        with pytest.raises(ValueError,match='exact unique native frame'):
            plot.associate(records[3],wrong,state,index,controls)
    wrong = deepcopy(camera)
    next(r for r in wrong if r['frame']==1400)['source_timestamps']['CAM_FRONT'] += .1
    with pytest.raises(ValueError,match='timestamps'):
        plot.associate(records[3],wrong,state,index,controls)


def test_archive_timestamp_mismatch_fails(actual):
    records = actual[0]
    state = plot.rows(plot.EPISODE/'states.jsonl')
    camera = plot.rows(plot.EPISODE/'camera_frames.jsonl')
    index = plot.rows(plot.TRIAL/'sampled_exact_runtime_inputs/index.jsonl')
    controls = plot.rows(plot.EPISODE/'learned_control.jsonl')
    index[3]['timing']['camera_stamps_ns'][0] += 1
    with pytest.raises(ValueError,match='archived input/frame'):
        plot.associate(records[3],camera,state,index,controls)


def test_same_axes_equal_meter_scale_no_clipping(actual):
    data = actual[1]
    for seq in plot.CASES:
        image, evidence = plot.case_plot(data,seq)
        assert image.size == (1200,740)
        for arm in ('A','B'):
            assert len(evidence[arm]['xy']['source_points']) == 64
            assert len(evidence[arm]['speed']['source_points']) == 64
            assert evidence[arm]['xy']['source_points'] == [(-p[1],p[0]) for p in data['cases'][str(seq)][arm]['xy_m']]
            assert evidence[arm]['speed']['source_points'] == list(zip(data['cases'][str(seq)]['time_s'],data['cases'][str(seq)][arm]['speed_mps']))
    assert plot.transform((0,0,480,480),(-40,40,-40,40),(0,0)) == (240,240)
    with pytest.raises(ValueError,match='do not clip'):
        plot.transform((0,0,480,480),(-40,40,-40,40),(41,0))


def test_overview_all149_no_smoothing(actual):
    image, evidence = plot.overview(actual[1])
    for arm in ('A','B'):
        assert evidence[arm]['source_points'] == list(zip(range(149),actual[1]['overview'][arm]['preview_mps']))


def test_no_image_resize_reencode_or_neural_imports():
    source = inspect.getsource(plot)
    for forbidden in ('import torch','import carla','.resize(','.thumbnail(','.crop(','.convert('):
        assert forbidden not in source
    assert 'shutil.copyfileobj(inp, out)' in source
    page = plot.gallery({'sequence':3,'frame':1400},[])
    assert 'width:640px;height:360px;max-width:none' in page
    assert 'B 실주행 영상이나 새 제어 ACK가 아닙니다' in page
    assert '독립적인6회 회복이 아닙니다' in plot.readme()


def test_actual_transformed_points_metadata_survives_png(actual,tmp_path,monkeypatch):
    monkeypatch.setattr(plot,'OUTPUT',tmp_path)
    data=actual[1]
    image,evidence=plot.case_plot(data,3)
    record=plot.save_plot('case_003.png',image,evidence,data['cases']['3'])
    plot.verify_rendered_points(data,record)
    assert record['logical_arrays_sha256'] == plot.jsha(data['cases']['3'])
    assert record['rendered_points_sha256'] == plot.jsha(evidence)
    changed=deepcopy(data['cases']['3']);changed['A']['speed_mps'][0] += .01
    assert record['logical_arrays_sha256'] != plot.jsha(changed)
    record['rendered_points']['A']['xy']['source_points'][0]=(1,2)
    with pytest.raises(ValueError,match='numerical coordinates'):
        plot.verify_rendered_points(data,record)


def test_input_sized_records_are_not_filtered(actual):
    with pytest.raises(ValueError,match='full fixed149'):
        plot.arrays(actual[0][1:])
    wrong=deepcopy(actual[0]);wrong[117]['new_A_or_B_control_ACK']=True
    with pytest.raises(ValueError,match='offline-only'):
        plot.arrays(wrong)
